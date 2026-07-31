import os


def analyze_workspace():
    root_dir = "/home/navantara/navantara"

    entry_points = ["run_backend_headless.py", "run_gui.py", "main.py"]
    core_dirs = ["src", "config", "firmware-asv"]

    categories = {
        "Main Entry Points": [],
        "Core Modules": [],
        "Tests & Utilities": [],
        "Uncategorized": [],
    }

    moves = []

    # Iterate only through the root level
    for item in sorted(os.listdir(root_dir)):
        path = os.path.join(root_dir, item)
        if os.path.isfile(path) and item.endswith(".py"):
            if item in entry_points:
                categories["Main Entry Points"].append(item)
            elif item.startswith("test_") or "sanity" in item or "utils" in item:
                categories["Tests & Utilities"].append(item)

                # Analyze if moving it breaks imports
                try:
                    with open(path, "r", encoding="utf-8") as f:
                        content = f.read()

                    warnings = []
                    if (
                        "sys.path.append" in content
                        or "os.path.dirname(__file__)" in content
                    ):
                        warnings.append(
                            "Contains sys.path manipulation which might break if depth is changed."
                        )

                    new_loc = (
                        f"tests/{item}" if item.startswith("test_") else f"utils/{item}"
                    )
                    moves.append(
                        {"file": item, "new_loc": new_loc, "warnings": warnings}
                    )
                except Exception as e:
                    moves.append(
                        {
                            "file": item,
                            "new_loc": "UNKNOWN",
                            "warnings": [f"Could not read file: {e}"],
                        }
                    )
            else:
                categories["Uncategorized"].append(item)
        elif os.path.isdir(path) and item in core_dirs:
            categories["Core Modules"].append(f"{item}/ (Directory)")

    print("=" * 70)
    print("WORKSPACE ANALYSIS REPORT")
    print("=" * 70)

    for cat, items in categories.items():
        if items:
            print(f"\n[{cat}]")
            for i in items:
                print(f"  - {i}")

    print("\n" + "=" * 70)
    print("RESTRUCTURING RECOMMENDATIONS (Tests & Utilities)")
    print("=" * 70)

    if not moves:
        print("No files identified for restructuring.")
    else:
        for m in moves:
            print(f"\nFile: {m['file']}")
            print(f"  -> Proposed Target : {m['new_loc']}")
            if m["warnings"]:
                for w in m["warnings"]:
                    print(f"  ⚠️ WARNING       : {w}")
            else:
                print("  ✅ Safe to move.")


if __name__ == "__main__":
    analyze_workspace()
