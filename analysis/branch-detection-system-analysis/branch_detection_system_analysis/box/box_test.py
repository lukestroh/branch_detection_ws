def main():
    mybox = BoxClient()
    print(mybox.client.user().get())
    """
    creates one level of subfolders in box and uploads files
    """
    root_dir = os.path.join(
        os.path.expanduser("~"), "branch_detection_ws", "bags", "2025_ToFBranchDetection",
    )
    # root_dir = os.path.join("/media/luke/T7 Shield", "luke")
    folders = glob.glob("bds*", root_dir=root_dir, recursive=True)
    # processed_root = os.path.join(root_dir, "uploaded")  # destination for moved folders
    # os.makedirs(processed_root, exist_ok=True)

    for folder in sorted(folders):
        subfolder_path = os.path.join(root_dir, folder)
        subfolder_id = None
        try:
            subfolder = mybox.client.folder(box_environments["2025_ToFBranchDetection"]).create_subfolder(folder)
            subfolder_id = subfolder.id
        except Exception as e:
            # existing folder: get its id
            try:
                subfolder_id = e.context_info["conflicts"][0]["id"]
                subfolder = mybox.get_folder(_id=subfolder_id)
                print(f"Folder {subfolder.name} already exists")
            except Exception:
                # fallback: try to look it up
                subfolder = mybox.get_folder_by_name(folder, parent_id=box_environments["2025_ToFBranchDetection"])
                subfolder_id = subfolder.id

        files = glob.glob("**/*.*", root_dir=subfolder_path, recursive=True)

        # track whether any non-conflict failures happened
        had_failure = False
        for file in files:
            file_path = os.path.join(subfolder_path, file)
            try:
                mybox.upload_file(file_path, _id=subfolder_id)
                print(f"Uploaded: {file_path}")
            except Exception as e:
                # If Box reports an item conflict (already exists), treat as OK/skip
                is_conflict = False
                try:
                    # many Box SDK exceptions include context_info with "conflicts"
                    if hasattr(e, "context_info") and e.context_info:
                        if "conflicts" in e.context_info:
                            is_conflict = True
                except Exception:
                    pass

                # another common check: a Box API 409 status (BoxAPIException may expose .status)
                try:
                    if not is_conflict and hasattr(e, "status") and getattr(e, "status") == 409:
                        is_conflict = True
                except Exception:
                    pass

                if is_conflict:
                    # file already exists on Box; skip and continue
                    try:
                        existing_name = e.context_info["conflicts"]["name"]
                    except Exception:
                        existing_name = os.path.basename(file_path)
                    print(f"Item {existing_name} already exists on Box — skipping.")
                else:
                    # a real failure we didn't expect; log and mark failure
                    print(f"Failed to upload {file_path}: {e}")
                    had_failure = True
                    # if you want to stop trying more files in this folder on first failure, uncomment next line:
                    # break

        # If no real failures (either uploaded or already existed), move the directory
        if not had_failure:
            dest = os.path.join(processed_root, os.path.basename(subfolder_path))
            # if destination exists, add timestamp suffix
            if os.path.exists(dest):
                timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                dest = f"{dest}_{timestamp}"
            try:
                shutil.move(subfolder_path, dest)
                print(f"Moved folder {subfolder_path} -> {dest}")
            except Exception as e:
                print(f"Failed to move {subfolder_path} to {dest}: {e}")
        else:
            print(f"Not moving {subfolder_path} because some uploads failed; will retry next run.")

    return