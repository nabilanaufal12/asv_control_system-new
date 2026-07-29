import os

def analyze_gitignore():
    workspace = "/home/navantara/navantara"
    gitignore_path = os.path.join(workspace, ".gitignore")
    
    best_practices = [
        "__pycache__/",
        "*.pyc",
        "*.pyo",
        ".venv/",
        "venv/",
        "env/",
        "logs/",
        "*.engine",
        "*.DS_Store"
    ]
    
    existing_rules = set()
    gitignore_exists = os.path.exists(gitignore_path)
    if gitignore_exists:
        with open(gitignore_path, 'r') as f:
            for line in f:
                stripped = line.strip()
                if stripped and not stripped.startswith("#"):
                    existing_rules.add(stripped)
                    
    missing_rules = [rule for rule in best_practices if rule not in existing_rules]
    
    # Scan for junk files in the workspace
    found_junk = set()
    for root, dirs, files in os.walk(workspace):
        # Exclude checking inside .git or virtual environments to save time
        if ".git" in root or ".venv" in root or "venv" in root:
            continue
        
        if "__pycache__" in dirs:
            found_junk.add("__pycache__/")
        if "logs" in dirs and root == workspace:
            found_junk.add("logs/")
            
        for f in files:
            if f.endswith(".pyc"):
                found_junk.add("*.pyc")
            elif f.endswith(".pyo"):
                found_junk.add("*.pyo")
            elif f.endswith(".engine"):
                found_junk.add("*.engine")
            elif f == ".DS_Store":
                found_junk.add("*.DS_Store")
                
    # Check for env directories at root
    for env_dir in [".venv", "venv", "env"]:
        if os.path.isdir(os.path.join(workspace, env_dir)):
            found_junk.add(f"{env_dir}/")
            
    print("==================================================")
    print(" GITIGNORE ANALYSIS REPORT")
    print("==================================================")
    
    print(f"\n[1] Current .gitignore Status")
    if gitignore_exists:
        print(f"  - Found: Yes")
        print(f"  - Existing Valid Rules: {len(existing_rules)}")
    else:
        print(f"  - Found: No")
    
    print(f"\n[2] Found Untracked Junk/Temporary Files")
    if found_junk:
        for j in sorted(list(found_junk)):
            print(f"  - {j}")
    else:
        print("  - None detected")
        
    print(f"\n[3] Proposed Additions to .gitignore")
    if missing_rules:
        for p in missing_rules:
            if p in found_junk:
                print(f"  + {p}  (⚠️ DETECTED IN WORKSPACE)")
            else:
                print(f"  + {p}")
    else:
        print("  - None (All best practices are already covered)")
        
    print("\n==================================================")

if __name__ == "__main__":
    analyze_gitignore()
