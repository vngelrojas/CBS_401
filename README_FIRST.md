# How to use dev container
- Download and install Docker
- In VSCODE, get the Dev Containers extension
- Make sure Docker is running
- Click Remote window button
  - ![alt text](/.devcontainer/remoteButtonImage.png)
- Click "Reopen in Container"
  - ![alt text](/.devcontainer/menuImage.png)
- The container should start building
  - If prompted to select a c++, just pick scan for kits option (Ignore this if you dont see that prompt)
- Download C++ and Python extensions
- Follow the Build and Run section in README.md
  - Note: Anything related Python will not work. I need to update the Dockerfile and scripts for that
<!-- - Docker was being stupid/I couldnt figure out how to install the requirements in the Dockerfile so run this to get python stuff
  - ```./.devcontainer/getPythonReqs.sh```
  - Should only have to do this the very first time you start dev container
- Follow the build directions in ITA-CBS-401Group/README
- If any python scripts dont work
  - In ITA-CBS2-401Group do ```source /workspaces/ITA-CBS_dev-main/python/myenv/bin/activate``` -->