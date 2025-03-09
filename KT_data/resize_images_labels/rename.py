import os
import datetime

def rename_files_in_directory(directory):
    today_date = datetime.datetime.today().strftime('%Y.%m.%d')
    
    for root, _, files in os.walk(directory):
        for file in files:
            old_path = os.path.join(root, file)
            file_name, file_ext = os.path.splitext(file)
            new_name = f"{today_date}_{file_name}{file_ext}"
            new_path = os.path.join(root, new_name)
            
            try:
                os.rename(old_path, new_path)
                print(f'Renamed: {old_path} -> {new_path}')
            except Exception as e:
                print(f'Error renaming {old_path}: {e}')

if __name__ == "__main__":
    folder_path = input("Enter the folder path: ").strip()
    if os.path.exists(folder_path) and os.path.isdir(folder_path):
        rename_files_in_directory(folder_path)
    else:
        print("Invalid folder path!")
