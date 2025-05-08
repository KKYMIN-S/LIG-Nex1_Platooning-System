# rasspberry_VSCode-install

## 이전 설치 시도 완전 제거
- sudo apt remove --purge code code-insiders code-oss
- sudo apt autoremove
- sudo rm -rf /var/lib/apt/lists/*

## APT 기본 설정 리셋
- sudo sed -i '/Default-Release/s/^/\\/\\//' /etc/apt/apt.conf.d/*
 
## 저장소 목록 갱신
- sudo apt update

## Buster Backports 활성화
- echo "deb http://deb.debian.org/debian buster-backports main contrib non-free" \
| sudo tee /etc/apt/sources.list.d/debian-buster-backports.list

- sudo apt update

## libxkbfile1 등의 의존성 최신화
- sudo apt -t buster-backports install libxkbfile1 libxkbcommon0

## Microsoft GPG 키와 저장소 설정
- wget -qO- https://packages.microsoft.com/keys/microsoft.asc \
| gpg --dearmor > microsoft.gpg
- sudo install -o root -g root -m644 microsoft.gpg /etc/apt/trusted.gpg.d/
- rm microsoft.gpg
- echo "deb [arch=armhf] https://packages.microsoft.com/repos/code stable main" \
| sudo tee /etc/apt/sources.list.d/vscode.list
- sudo apt update

## VS Code 설치
- sudo apt install code
