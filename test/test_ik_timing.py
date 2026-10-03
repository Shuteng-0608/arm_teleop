import ast
import csv
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

from core.ik_timing import (SERVER_TIMES, STAGES, PAPER_STAGES, PAPER_TIMES,
                           PAPER_FLAGS, RESPONSE_FIELDS, TIMES, TimingRecorder, capture, summarize)

ROOT = Path(__file__).resolve().parents[1]


def response(**overrides):
    values = {name: 10.0 for name in SERVER_TIMES}
    values.update(success=True, message="A1_minimum_jv:fallback_baseline:none",
                  timing_valid=True, target_is_moving=True, timing_runtime_mode="execution",
                  solution=[0, -.5, 0, 1, 0, -.3, 0], new_arm_angle=0.0)
    values.update(overrides)
    return SimpleNamespace(**values)


class TimingTests(unittest.TestCase):
    def test_fields_match_service_and_cpp_mapping(self):
        srv = (ROOT/'srv/ArmIK.srv').read_text().split('---')[1]
        cpp = (ROOT/'src/ik_service.cpp').read_text()
        for field in SERVER_TIMES + PAPER_TIMES:
            self.assertIn('float64 '+field, srv)
            self.assertIn('res.'+field+' =', cpp)

    def test_groups_zero_missing_startup_and_failure(self):
        frame = SimpleNamespace(index=0, timestamp=0.)
        rows = [capture(frame, 'B', 100, response(), startup=True),
                capture(frame, 'B', 20, response(phi2_us=0.)),
                capture(frame, 'B', 30, response(message='B:hold_previous:budget')),
                capture(frame, 'B', 40, response(target_is_moving=False)),
                capture(frame, 'B', 50, error='RPC failed'),
                capture(frame, 'B', 60, SimpleNamespace(success=True, message='legacy'))]
        s = summarize(rows)['groups']
        self.assertEqual(s['all_calls']['calls'], 6)
        self.assertEqual(s['moving_non_hold']['calls'], 1)
        self.assertEqual(s['moving_hold']['calls'], 1)
        self.assertEqual(s['static_calls']['calls'], 1)
        self.assertEqual(s['failed_calls']['calls'], 1)
        self.assertEqual(s['unavailable_timing']['calls'], 1)
        self.assertIsNone(s['moving_non_hold']['metrics_us']['phi2_us']['mean'])
        self.assertEqual(s['all_calls']['metrics_us']['phi1_us']['missing_count'], 2)
        self.assertAlmostEqual(s['all_calls']['metrics_us']['ik_latency_us']['p50'], 45.)
        self.assertAlmostEqual(s['all_calls']['metrics_us']['ik_latency_us']['p95'], 90.)
        self.assertAlmostEqual(s['all_calls']['metrics_us']['ik_latency_us']['p99'], 98.)

    def test_bad_server_values_not_recorded_as_zero(self):
        for value in (float('nan'), -1., None):
            r = capture(SimpleNamespace(index=1,timestamp=.1),'B',1,response(phi1_us=value))
            self.assertFalse(r['timing_valid'])
            self.assertEqual(r['phi1_us'], '')

    def test_sidecar_roundtrip_and_no_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory)/'run.csv')
            recorder = TimingRecorder(path)
            recorder.record(capture(SimpleNamespace(index=1,timestamp=.1),'B',10,response()))
            recorder.close()
            recorder.close()
            with open(recorder.path) as f:
                self.assertEqual(len(list(csv.DictReader(f))), 1)
            with open(recorder.summary_path) as f:
                self.assertEqual(json.load(f)['groups']['all_calls']['calls'], 1)
            with open(recorder.summary_csv_path) as f:
                self.assertEqual(len(list(csv.DictReader(f))), 7*len(TIMES))
            with self.assertRaises(FileExistsError):
                TimingRecorder(path)

    def solver(self, service):
        # Execute the real entry-point class with only ROS I/O boundaries stubbed.
        # This tests client logic locally; it is NOT a ROS serialization test.
        tree = ast.parse((ROOT/'vptele/playback_right_teleop.py').read_text())
        nodes = [x for x in tree.body if isinstance(x, (ast.ClassDef,ast.FunctionDef))
                 and x.name in ('OnlineRedundancySolver','result_row','output_fields')]
        namespace = dict(time=SimpleNamespace(monotonic=mock.Mock(side_effect=[2.,2.001])),
                         capture=capture, RESPONSE_FIELDS=RESPONSE_FIELDS,
                         rounded_solver_state=lambda q: tuple(q),
                         make_ik_request=lambda *a: a,
                         validate_online_solution=lambda *a: (tuple(a[0]),SimpleNamespace(maximum_step=0.,maximum_velocity=0.)))
        exec(compile(ast.Module(body=nodes,type_ignores=[]),'playback_client_logic','exec'),namespace)
        args=SimpleNamespace(ik_method='A1_minimum_jv',maximum_step=.03,maximum_velocity=.8)
        solver=namespace['OnlineRedundancySolver'](service,args,[0,-.5,0,1,0,-.3,0],0.)
        solver.timing_recorder=mock.Mock()
        return solver,namespace

    def test_real_client_records_before_solution_rejection(self):
        for res in (response(success=False),response(message='bad status')):
            solver,_=self.solver(SimpleNamespace(call=lambda request:res))
            with self.assertRaises(RuntimeError):
                solver.solve(SimpleNamespace(index=7,timestamp=.2),[0]*7)
            row=solver.timing_recorder.record.call_args[0][0]
            self.assertAlmostEqual(row['ik_latency_us'],1000.)
            self.assertEqual(row['frame_index'],7)
            self.assertIsNone(solver.previous_output_joints)

    def test_real_client_records_rpc_failure(self):
        solver,_=self.solver(SimpleNamespace(call=mock.Mock(side_effect=RuntimeError('transport'))))
        with self.assertRaisesRegex(RuntimeError,'transport'):
            solver.solve(SimpleNamespace(index=1,timestamp=.2),[0]*7)
        self.assertFalse(solver.timing_recorder.record.call_args[0][0]['timing_valid'])

    def test_success_csv_fields_match(self):
        solver,ns=self.solver(SimpleNamespace(call=lambda request: response()))
        frame=SimpleNamespace(index=1,timestamp=.2)
        result=solver.solve(frame,[0]*7)
        row=ns['result_row']('input',SimpleNamespace(file_sha256='hash'),frame,[0]*7,result)
        self.assertEqual(set(row),set(ns['output_fields']()))
        self.assertEqual(row['phi1_us'],10.)
        self.assertEqual(row['timing_runtime_mode'],'execution')

    def paper_response(self, skipped=False, **overrides):
        values = dict(zip(PAPER_STAGES, [10., 1., 0. if skipped else 2.,
                                         0. if skipped else 3., 1.]))
        values.update(zip(PAPER_FLAGS, [True, True, not skipped, not skipped, True]))
        values.update(timing_schema_version=2, paper_timing_valid=True,
                      selector_other_us=4., selector_mode='normal', wrist_skip_reason='none')
        values['selector_elapsed_us'] = sum(values[key] for key in PAPER_TIMES)
        values.update(overrides)
        return response(**values)

    def test_paper_executed_statistics_and_sum(self):
        frame = SimpleNamespace(index=1, timestamp=.1)
        # An executed search counts even when the result is fallback_baseline.
        rows = [capture(frame, 'P', 50, self.paper_response()),
                capture(frame, 'P', 50, self.paper_response(skipped=True)),
                capture(frame, 'P', 50, response())]
        self.assertTrue(rows[0]['paper_timing_valid'])
        self.assertFalse(rows[2]['paper_timing_valid'])
        s = summarize(rows)['groups']['moving_non_hold']
        m = s['metrics_us']['stage4_wrist_candidates_us']
        self.assertEqual((m['count'], m['skipped_count'], m['missing_count']), (1, 1, 1))
        self.assertEqual(m['mean'], 3.)
        self.assertEqual(m['mean_all_calls'], 1.5)
        self.assertEqual(m['execution_rate'], .5)
        self.assertAlmostEqual(sum(s['paper_time_fractions'].values()), 1.)

    def test_bad_paper_data_preserves_total_timing(self):
        for changes in ({'selector_other_us': 100.}, {'stage3_executed': False},
                        {'stage1_baseline_us': float('nan')}, {'stage1_executed': None},
                        {'stage4_wrist_candidates_us': -1.}):
            row = capture(SimpleNamespace(index=1,timestamp=.1), 'P', 50,
                          self.paper_response(**changes))
            self.assertTrue(row['timing_valid'])
            self.assertFalse(row['paper_timing_valid'])
            self.assertTrue(row['paper_timing_error'])

    def test_true_zero_duration_is_not_a_skipped_stage(self):
        res = self.paper_response(stage4_wrist_candidates_us=0., selector_elapsed_us=18.)
        row = capture(SimpleNamespace(index=1,timestamp=.1), 'P', 50, res)
        m = summarize([row])['groups']['all_calls']['metrics_us']['stage4_wrist_candidates_us']
        self.assertEqual(m['count'], 1)
        self.assertEqual(m['mean'], 0.)
        self.assertEqual(m['execution_rate'], 1.)


if __name__ == '__main__':
    unittest.main()
