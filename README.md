# RaspberryPi-Docker-Install
Install Docker

- sudo apt update
- sudo apt upgrade

1단계 - 필수 패키지 설치
sudo apt install ca-certificates
sudo apt install curl
sudo apt install gnupg
sudo apt install lsb-release

2단계 - Docker GPG 키 등록
sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/raspbian/gpg | \ sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg

3단계 - Docker 저장소 추가
echo \ "deb [arch=armhf signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/raspbian \ buster stable" | \ sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

4단계 - Docker 설치
sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io

5단계 - Docker 설치 확인 및 권한 설정
docker --version
sudo usermod -aG docker $USER
reboot

6단계 - 정상 설치 확인
docker run hello-world
