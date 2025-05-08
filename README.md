# rasspberry_VSCode-install
- sudo apt update

- sudo apt install -y software-properties-common apt-transport-https wget

- wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > microsoft.gpg

- sudo install -o root -g root -m 644 microsoft.gpg /etc/apt/trusted.gpg.d/

- rm microsoft.gpg

- echo "deb [arch=arm64] https://packages.microsoft.com/repos/code stable main" | \
  sudo tee /etc/apt/sources.list.d/vscode.list


- sudo apt update

- sudo apt install code
