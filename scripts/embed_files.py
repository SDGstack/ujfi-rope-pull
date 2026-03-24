import os
import json

Import("env")

print(">>> embed_files.py script is running <<<")

# Source: src/Web/*
WEB_DIR = os.path.join(env["PROJECT_DIR"], "src", "Web")

# Output: include/web/*
OUT_DIR = os.path.join(env["PROJECT_DIR"], "include", "web")
os.makedirs(OUT_DIR, exist_ok=True)


def sanitize_name(subdir, filename):
    """Create variable name: subdir_filename"""
    name = (subdir + "_" + filename) if subdir else filename
    return name.replace(".", "_").replace("-", "_")


for root, dirs, files in os.walk(WEB_DIR):
    # Compute subdir relative to WEB_DIR
    rel_subdir = os.path.relpath(root, WEB_DIR)
    if rel_subdir == ".":
        rel_subdir = ""

    # Collect files to embed
    file_vars = []
    for file in files:
        if file.endswith((".html", ".js", ".css")):
            path = os.path.join(root, file)
            var_name = sanitize_name(rel_subdir, file)

            with open(path, "r", encoding="utf-8") as f:
                content = f.read()
            # Use JSON for safe escaping
            file_vars.append((var_name, json.dumps(content)))

    if not file_vars:
        continue

    # Output header file: one per subdir
    header_name = (rel_subdir.replace(os.sep, "_") if rel_subdir else "root") + ".h"
    out_file = os.path.join(OUT_DIR, header_name)

    # Optional: skip if unchanged
    if os.path.exists(out_file):
        latest_source_mtime = max(os.path.getmtime(os.path.join(root, f)) for f in files)
        if os.path.getmtime(out_file) >= latest_source_mtime:
            continue

    print(f"Generating {out_file}")

    # Write header
    with open(out_file, "w", encoding="utf-8") as f:
        f.write("#pragma once\n#include <Arduino.h>\n\n")
        for var_name, string_literal in file_vars:
            f.write(f"const String {var_name} = {string_literal};\n")