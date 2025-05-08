# rasspberry_VSCode-install
- sudo apt update

- sudo apt install -y software-properties-common apt-transport-https wget

- wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg

- sudo install -o root -g root -m 644 packages.microsoft.gpg /usr/share/keyrings/
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/vscode stable main" \
  | sudo tee /etc/apt/sources.list.d/vscode.list

- sudo rm packages.microsoft.gpg

- sudo apt update

- sudo apt install code
