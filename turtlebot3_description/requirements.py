import subprocess

# Execute a terminal command
with open("requirements.txt", "r") as f:
    # Read the entire contents of the file
    contents = f.read()

subprocess.call(contents, shell=True)
