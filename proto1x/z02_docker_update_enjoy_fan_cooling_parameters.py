#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import subprocess
import re
try:
    from ruamel.yaml import YAML
except ModuleNotFoundError:
    print("ruamel.yaml not found. Installing...")
    subprocess.check_call([sys.executable, "-m", "pip", "install", "ruamel.yaml"])
    from ruamel.yaml import YAML  # retry import


def load_yaml(file_path):
    """Load YAML preserving order and comments."""
    yaml = YAML()
    try:
        with open(file_path, 'r') as fp:
            data = yaml.load(fp)
    except FileNotFoundError:
        print(f"Error: File not found: {file_path}", file=sys.stderr)
        sys.exit(1)
    return yaml, data


def find_ros_parameters(node):
    """
    Recursively search for the first 'ros__parameters' dict
    within the loaded YAML structure.
    """
    if not isinstance(node, dict):
        return None
    if 'ros__parameters' in node:
        return node['ros__parameters']
    for value in node.values():
        result = find_ros_parameters(value)
        if result is not None:
            return result
    return None


def get_tf_translation_once():
    """Run tf2_echo once and parse the first Translation line."""
    cmd = ['ros2', 'run', 'tf2_ros', 'tf2_echo', 'proto1x/world', 'proto1x/base_link']
    proc = subprocess.Popen(cmd,
                            stdout=subprocess.PIPE,
                            stderr=subprocess.DEVNULL,
                            text=True)
    x = y = None
    for line in proc.stdout:
        # match any line containing "Translation:"
        if 'Translation:' in line:
            m = re.search(r'Translation:\s*\[([^]]+)\]', line)
            if m:
                parts = m.group(1).split(',')
                x = float(parts[0].strip())
                y = float(parts[1].strip())
            break
    proc.kill()
    if x is None or y is None:
        raise RuntimeError("Could not parse TF translation")
    return x, y


def cast_value(orig_value, new_str):
    """Cast input string to the type of orig_value."""
    if isinstance(orig_value, bool):
        return new_str.lower() in ('true', '1', 'yes', 'on')
    if isinstance(orig_value, int):
        return int(new_str)
    if isinstance(orig_value, float):
        return float(new_str)
    return new_str  # fallback to string

def copy_to_install(src_path):
    """Copy the updated config file to the install share path."""
    # Build destination path
    dst_path = os.path.expanduser(
        '~/workspace/riza/ros2_ws/install/enjoy_fan_cooling/'
        'share/enjoy_fan_cooling/config/enjoy_fan_cooling.yaml'
    )
    dst_dir = os.path.dirname(dst_path)

    try:
        os.makedirs(dst_dir, exist_ok=True)
        shutil.copy2(src_path, dst_path)
        print(f"\nCopied to install path:\n  {dst_path}")
    except PermissionError as e:
        print(f"Error: Permission denied copying to {dst_path}: {e}", file=sys.stderr)
    except FileNotFoundError as e:
        print(f"Error: Destination not found {dst_dir}: {e}", file=sys.stderr)
    except OSError as e:
        print(f"Error: Failed to copy to {dst_path}: {e}", file=sys.stderr)

def main():
    file_path = os.path.expanduser('~/workspace/riza/ros2_ws/src/behavior/enjoy_fan_cooling/config/enjoy_fan_cooling.yaml')

    # --- print original file ---
    print("=== Original hoge.yaml ===")
    try:
        with open(file_path, 'r') as f:
            print(f.read().rstrip())
    except FileNotFoundError:
        print(f"Error: File not found: {file_path}", file=sys.stderr)
        sys.exit(1)
    print("==========================\n")

    # load YAML
    yaml, data = load_yaml(file_path)

    # find parameters
    params = find_ros_parameters(data)
    if params is None:
        print("Error: 'ros__parameters' section not found.", file=sys.stderr)
        sys.exit(1)

    # ask whether to fetch TF
    ans = input("Fetch 3 frames of TF for goal_position? [Y/n]: ").strip().lower()
    if ans in ('', 'y', 'yes'):
        xs = []
        ys = []
        print("\nReceiving 3 frames:")
        for i in range(1, 4):
            try:
                x, y = get_tf_translation_once()
            except Exception as e:
                print(f"[Frame {i}] Error: {e}", file=sys.stderr)
                sys.exit(1)
            print(f"[Frame {i}] Translation: x={x}, y={y}")
            xs.append(x)
            ys.append(y)
        avg_x = sum(xs) / len(xs)
        avg_y = sum(ys) / len(ys)
        print(f"\nAverage Translation: x={avg_x}, y={avg_y}\n")
        params['goal_position_x'] = avg_x
        params['goal_position_y'] = avg_y

    # prompt for other parameters
    print(f"\nEditing other parameters in: {file_path}\n"
          "Press Enter to keep current value.\n")
    for key, orig_value in params.items():
        if key in ('goal_position_x', 'goal_position_y'):
            continue
        prompt = f"{key} [{orig_value}]: "
        user_input = input(prompt).strip()
        if user_input:
            try:
                params[key] = cast_value(orig_value, user_input)
            except ValueError as e:
                print(f"Invalid input for {key}: {e}", file=sys.stderr)

    # write back
    with open(file_path, 'w') as fp:
        yaml.dump(data, fp)

    # --- print updated file ---
    print("=== Updated hoge.yaml ===")
    with open(file_path, 'r') as f:
        print(f.read().rstrip())
    print("==========================")

    print("\nParameters updated successfully.")


if __name__ == '__main__':
    main()

