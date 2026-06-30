import pymeshlab
import os

# ================= Configuration =================
# Automatically get the current directory where the script is running
TARGET_DIR = os.path.dirname(os.path.abspath(__file__))

# Target maximum face count (MuJoCo recommends < 200k, set to 150k for safety)
MAX_FACES = 150000 

# Overwrite original files? (True: Overwrite, False: Save as _fixed.stl)
OVERWRITE = True 
# ===============================================

def process_stl(file_path):
    # Create a new MeshSet
    ms = pymeshlab.MeshSet()
    
    try:
        filename = os.path.basename(file_path)
        # Skip the script itself if it's in the same folder
        if filename.endswith('.py'):
            return

        print(f"Reading: {filename} ...")
        ms.load_new_mesh(file_path)
        
        # Get current mesh statistics
        m = ms.current_mesh()
        face_count = m.face_number()
        print(f"  - Original faces: {face_count}")

        # Flag to determine if saving is needed
        need_save = False

        # 1. Check face count and decimate if necessary
        if face_count > MAX_FACES:
            print(f"  - Too many faces (> {MAX_FACES}), decimating...")
            # Apply Quadric Edge Collapse Decimation filter
            ms.apply_filter('meshing_decimation_quadric_edge_collapse', 
                            targetfacenum=MAX_FACES, 
                            preserveboundary=True, 
                            preservenormal=True)
            new_face_count = ms.current_mesh().face_number()
            print(f"  - Decimation complete: {new_face_count} faces")
            need_save = True
        else:
            # Even if face count is low, force save to fix potential "ASCII file" errors
            # This ensures the file is converted to Binary STL
            need_save = True

        # 2. Save the file
        if need_save:
            output_path = file_path if OVERWRITE else file_path.replace('.STL', '_fixed.STL').replace('.stl', '_fixed.stl')
            
            # save_current_mesh handles binary STL conversion automatically
            try:
                ms.save_current_mesh(output_path, binary=True)
            except TypeError:
                # Fallback for older pymeshlab versions
                ms.save_current_mesh(output_path)
                
            print(f"  - Saved to: {os.path.basename(output_path)} (Binary STL)")
        else:
            print("  - No changes needed.")

    except Exception as e:
        print(f"  - Error processing file: {e}")
    print("-" * 30)

def main():
    print(f"Processing directory: {TARGET_DIR}")
    
    # List all STL files in the current directory
    files = [f for f in os.listdir(TARGET_DIR) if f.lower().endswith('.stl')]
    
    if not files:
        print("No STL files found in current directory. Please run this script inside the 'meshes' folder.")
        return

    print(f"Found {len(files)} STL files, starting process...")
    
    for f in files:
        full_path = os.path.join(TARGET_DIR, f)
        process_stl(full_path)
    
    print("\nAll done!")

if __name__ == "__main__":
    main()
