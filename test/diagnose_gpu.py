#!/usr/bin/env python3
"""
MuJoCo GPU 渲染诊断脚本。

检查：
  1. 系统是否有 NVIDIA GPU
  2. OpenGL 当前使用哪个渲染器（GPU 还是软件）
  3. MuJoCo 实际使用的 OpenGL 后端
  4. 渲染性能基准测试

用法：
    python diagnose_gpu.py
"""

import subprocess
import sys
import os


def run(cmd: str, timeout: int = 10) -> str:
    """Run a shell command and return stdout, or '' on failure."""
    try:
        r = subprocess.run(cmd, shell=True, capture_output=True, text=True,
                           timeout=timeout)
        return (r.stdout + r.stderr).strip()
    except Exception:
        return ""


def section(title: str):
    print()
    print("=" * 60)
    print(f"  {title}")
    print("=" * 60)


# ---------------------------------------------------------------------------
# 1. NVIDIA GPU 硬件检测
# ---------------------------------------------------------------------------
section("1. NVIDIA GPU 硬件")

nvidia_smi = run("nvidia-smi -L 2>/dev/null")
if nvidia_smi:
    print(f"  [OK] ✓ 检测到 NVIDIA GPU:")
    for line in nvidia_smi.splitlines():
        if line.strip():
            print(f"       {line.strip()}")
else:
    print(f"  [WARN] nvidia-smi 不可用，检查其他方式...")

lspci = run("lspci 2>/dev/null | grep -i 'vga\|3d\|display'")
if lspci:
    print(f"  PCI 显示设备:")
    for line in lspci.splitlines():
        if line.strip():
            print(f"       {line.strip()}")
else:
    print(f"  (lspci 不可用)")

# ---------------------------------------------------------------------------
# 2. NVIDIA 驱动状态
# ---------------------------------------------------------------------------
section("2. NVIDIA 驱动")

nvidia_driver = run("nvidia-smi 2>/dev/null | head -5")
if nvidia_driver and "Driver Version" in nvidia_driver:
    for line in nvidia_driver.splitlines():
        print(f"  {line}")
else:
    print(f"  [FAIL] ✗  NVIDIA 驱动未加载或未安装")
    print(f"         → sudo apt install nvidia-driver-<version>")

# Check kernel module
lsmod = run("lsmod 2>/dev/null | grep -i nvidia")
if lsmod:
    print(f"  [OK] ✓  nvidia 内核模块已加载")
else:
    print(f"  [WARN] nvidia 内核模块未加载 (可能使用 nouveau 开源驱动)")

# ---------------------------------------------------------------------------
# 3. OpenGL 渲染器
# ---------------------------------------------------------------------------
section("3. OpenGL 渲染器")

glxinfo = run("glxinfo -B 2>/dev/null")
if glxinfo:
    for line in glxinfo.splitlines():
        line = line.strip()
        if any(k in line for k in ["OpenGL renderer", "OpenGL vendor",
                                     "OpenGL version", "Device:", "Accelerated"]):
            print(f"  {line}")

    # 关键判断
    if "llvmpipe" in glxinfo or "softpipe" in glxinfo:
        print()
        print(f"  ⚠ ⚠ ⚠  [CRITICAL]  当前使用软件渲染 (llvmpipe/softpipe)！")
        print(f"             MuJoCo viewer 会用 CPU 渲染，极慢。")
        print(f"             这很可能就是你的问题。")
    elif "NVIDIA" in glxinfo:
        print()
        print(f"  [OK] ✓  OpenGL 正在使用 NVIDIA GPU 硬件加速")
    elif "Mesa" in glxinfo and "Intel" in glxinfo:
        print(f"  [INFO] 使用 Intel 集成显卡 (Mesa)")
    elif "AMD" in glxinfo or "Radeon" in glxinfo:
        print(f"  [INFO] 使用 AMD GPU")
else:
    print(f"  [WARN] glxinfo 不可用")
    print(f"         → sudo apt install mesa-utils")

# Also check EGL (MuJoCo might use EGL instead of GLX)
print()
print(f"  EGL 信息:")
eglinfo = run("eglinfo -B 2>/dev/null | head -10")
if eglinfo:
    for line in eglinfo.splitlines():
        if any(k in line for k in ["renderer", "vendor", "version", "Device"]):
            print(f"  {line.strip()}")
else:
    print(f"  (eglinfo 不可用)")

# ---------------------------------------------------------------------------
# 4. MuJoCo 实际使用的 OpenGL 信息
# ---------------------------------------------------------------------------
section("4. MuJoCo OpenGL 上下文")

try:
    import mujoco
    print(f"  MuJoCo 版本: {mujoco.__version__ if hasattr(mujoco, '__version__') else '未知'}")

    # Try to query what MuJoCo uses for rendering
    # MuJoCo >= 3.x uses its own OpenGL context management
    try:
        # Check if headless — this would indicate no display
        display = os.environ.get("DISPLAY", "")
        print(f"  DISPLAY      = {display if display else '(未设置 → 可能使用 EGL/OSMesa)'}")

        mujoco_gl = os.environ.get("MUJOCO_GL", "")
        print(f"  MUJOCO_GL    = {mujoco_gl if mujoco_gl else '(未设置 → 默认: glx 或 egl)'}")

        # Check common env vars
        egl_platform = os.environ.get("EGL_PLATFORM", "")
        print(f"  EGL_PLATFORM = {egl_platform if egl_platform else '(未设置)'}")

    except Exception:
        pass

    # Try creating a minimal render context to see which backend is used
    print()
    print(f"  测试 MuJoCo 离屏渲染...")
    try:
        # Load a minimal model or use the actual one
        model_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "..", "model", "pangu_all_right.xml"
        )
        model_path = os.path.normpath(model_path)

        if not os.path.exists(model_path):
            # fallback: create a minimal inline model
            xml = '''
            <mujoco>
              <worldbody>
                <light pos="0 0 5"/>
                <geom type="box" size="0.5 0.5 0.5"/>
                <body pos="0 0 1">
                  <joint name="j1" type="hinge" axis="0 0 1"/>
                  <geom type="cylinder" size="0.05 0.3" rgba="1 0 0 1"/>
                </body>
              </worldbody>
            </mujoco>
            '''
            import tempfile
            with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False) as f:
                f.write(xml)
                model_path = f.name

        model = mujoco.MjModel.from_xml_path(model_path)
        data = mujoco.MjData(model)

        # Time rendering
        import time
        renderer = mujoco.Renderer(model, 640, 480)
        mujoco.mj_forward(model, data)
        renderer.update_scene(data)

        t0 = time.perf_counter()
        for _ in range(100):
            renderer.render()
        t1 = time.perf_counter()
        elapsed = t1 - t0
        fps = 100 / elapsed

        print(f"  100 帧离屏渲染耗时: {elapsed:.3f}s  (≈ {fps:.0f} fps)")
        if fps > 500:
            print(f"  [OK] ✓  离屏渲染速度正常，GPU 加速已启用")
        elif fps > 100:
            print(f"  [INFO] 离屏渲染速度一般，可能是集成显卡")
        else:
            print(f"  [WARN] 离屏渲染较慢 ({fps:.0f} fps)，可能使用软件渲染")

        renderer.close()

        # Clean up temp file
        if 'tempfile' in dir() and os.path.exists(model_path):
            try:
                os.unlink(model_path)
            except Exception:
                pass

    except Exception as e:
        print(f"  [FAIL] MuJoCo 渲染测试失败: {e}")

except ImportError:
    print(f"  [FAIL] 无法导入 mujoco 包")
    print(f"         → pip install mujoco")

# ---------------------------------------------------------------------------
# 5. 诊断结论和修复建议
# ---------------------------------------------------------------------------
section("5. 诊断结论")

# Quick re-check the key indicator
glxinfo = run("glxinfo -B 2>/dev/null | grep 'OpenGL renderer'")
print()

if "NVIDIA" in glxinfo:
    print("  ✅ GPU 渲染正常。OpenGL 正在使用 NVIDIA 硬件加速。")
    print("     MuJoCo viewer 应该流畅。")
    print("     如果仍然卡顿，检查：")
    print("     - MUJOCO_GL 环境变量是否为 egl (某些平台用 glx 更稳定)")
    print("     - nvidia-smi 确认 GPU 利用率")
    print("     - 尝试 export MUJOCO_GL=glx 或 export MUJOCO_GL=egl")
elif "llvmpipe" in glxinfo or "softpipe" in glxinfo:
    print("  ❌ GPU 渲染未启用！OpenGL 使用 CPU 软件渲染 (llvmpipe)。")
    print("     这是 MuJoCo viewer 卡顿的根因。")
    print()
    print("  修复步骤：")
    print("  1. 确认 NVIDIA 驱动已安装：")
    print("     ubuntu-drivers devices")
    print("     sudo apt install nvidia-driver-535  (或推荐版本)")
    print()
    print("  2. 重启后验证：")
    print("     nvidia-smi")
    print("     glxinfo | grep 'OpenGL renderer'  # 应显示 NVIDIA")
    print()
    print("  3. 如果是双显卡笔记本，可能需要 prime-select：")
    print("     sudo prime-select nvidia")
    print("     然后重新登录")
elif glxinfo:
    print(f"  当前渲染器: {glxinfo}")
    print("  请确认这对应你的独立 GPU 而非集成显卡")
else:
    print("  无法确定 GPU 状态。")
    print("  安装 mesa-utils 获取更多信息：")
    print("    sudo apt install mesa-utils")
    print("  然后运行: glxinfo -B | grep 'OpenGL renderer'")


if __name__ == "__main__":
    pass
