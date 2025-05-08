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


- pi@raspberrypi:~ $ sudo apt install code
Reading package lists... Done
Building dependency tree       
Reading state information... Done
Some packages could not be installed. This may mean that you have
requested an impossible situation or if you are using the unstable
distribution that some required packages have not yet been created
or been moved out of Incoming.
The following information may help to resolve the situation:

The following packages have unmet dependencies:
 code : Depends: libxkbfile1 (>= 1:1.1.0) but 1:1.0.9-2 is to be installed
E: Unable to correct problems, you have held broken packages.
pi@raspberrypi:~ $ ^C


# New version

## 불필요한 backports 리스트 삭제
- sudo rm /etc/apt/sources.list.d/*buster-backports*.list
- sudo sed -i '/buster-backports/d' /etc/apt/sources.list
- sudo rm -rf /var/lib/apt/lists/*
- sudo apt update

- wget http://ftp.debian.org/debian/pool/main/libx/libxkbfile/libxkbfile1_1.1.0-2~bpo10+1_armhf.deb
- sudo dpkg -i libxkbfile1_1.1.0-2~bpo10+1_armhf.deb
- 만약 의존성 경고가 뜨면, sudo apt --fix-broken install
- sudo apt install code


- wget https://update.code.visualstudio.com/latest/armhf/deb -O vscode-armhf.deb
- sudo dpkg -i vscode-armhf.deb
- sudo apt --fix-broken install

- deb http://archive.raspberrypi.org/debian/ buster main ui

