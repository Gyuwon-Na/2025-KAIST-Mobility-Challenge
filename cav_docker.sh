#!/bin/bash

INTERFACE="wlo1"

echo "=========================================="
echo "CAV Docker 자동 배포 스크립트 (비밀번호 자동 입력)"
echo "=========================================="
echo ""
echo "경고: 이 스크립트는 비밀번호가 평문으로 포함되어 있습니다."
echo "보안에 주의하세요. 프로덕션 환경에서는 SSH 키를 사용하세요."
echo ""

echo "[1/4] sshpass 설치 확인 중..."
if ! command -v sshpass &> /dev/null; then
    echo "sshpass가 설치되어 있지 않습니다."
    read -p "지금 설치하시겠습니까? (y/n): " install_confirm
    if [ "$install_confirm" = "y" ]; then
        sudo apt-get update
        sudo apt-get install -y sshpass
        echo "sshpass 설치 완료"
    else
        echo "sshpass가 필요합니다. 스크립트를 종료합니다."
        exit 1
    fi
else
    echo "sshpass 설치 확인 완료"
fi

echo ""
echo "[2/4] 네트워크 인터페이스 확인..."
ip addr show | grep -E "^[0-9]+:"
echo ""
echo "현재 INTERFACE 설정: $INTERFACE"
read -p "인터페이스 이름이 맞습니까? (y/n): " confirm

if [ "$confirm" != "y" ]; then
    echo "스크립트를 종료합니다. 파일을 열어 INTERFACE 변수를 수정하세요."
    exit 1
fi

echo ""
echo "[3/4] IP 주소 할당 중..."
sudo ip addr add 192.168.10.55/24 dev $INTERFACE 2>/dev/null
if [ $? -eq 0 ]; then
    echo "192.168.10.55/24 할당 완료"
else
    echo "192.168.10.55/24 이미 할당됨 또는 오류"
fi

sudo ip addr add 192.168.11.55/24 dev $INTERFACE 2>/dev/null
if [ $? -eq 0 ]; then
    echo "192.168.11.55/24 할당 완료"
else
    echo "192.168.11.55/24 이미 할당됨 또는 오류"
fi

echo ""
echo "할당된 IP 확인:"
ip addr show $INTERFACE | grep inet

echo ""
echo "[4/4] 4개 터미널 자동 실행 중..."
sleep 1

gnome-terminal --title="CAV02 (192.168.11.221)" -- bash -c '
echo "=========================================="
echo "CAV02 Docker 자동 실행 (192.168.11.221)"
echo "=========================================="
echo ""
echo "자동 접속 및 실행 중..."
sshpass -p "1234" ssh -o StrictHostKeyChecking=no -t cav-12@192.168.11.221 "cd ~/kmc_cav_docker && echo \"1234\" | sudo -S docker compose build && echo \"1234\" | sudo -S docker compose up; exec bash"
'

gnome-terminal --title="CAV03 (192.168.10.196)" -- bash -c '
echo "=========================================="
echo "CAV03 Docker 자동 실행 (192.168.10.196)"
echo "=========================================="
echo ""
echo "자동 접속 및 실행 중..."
sshpass -p "1234" ssh -o StrictHostKeyChecking=no -t cav-13@192.168.10.196 "cd ~/kmc_cav_docker && echo \"1234\" | sudo -S docker compose build && echo \"1234\" | sudo -S docker compose up; exec bash"
'

gnome-terminal --title="CAV04 (192.168.10.71)" -- bash -c '
echo "=========================================="
echo "CAV04 Docker 자동 실행 (192.168.10.71)"
echo "=========================================="
echo ""
echo "자동 접속 및 실행 중..."
sshpass -p "1234" ssh -o StrictHostKeyChecking=no -t cav-14@192.168.10.71 "cd ~/kmc_cav_docker && echo \"1234\" | sudo -S docker compose build && echo \"1234\" | sudo -S docker compose up; exec bash"
'

gnome-terminal --title="BISA (192.168.11.133)" -- bash -c '
echo "=========================================="
echo "BISA Docker 자동 실행 (192.168.11.133)"
echo "=========================================="
echo ""
echo "자동 접속 및 실행 중..."
sshpass -p " " ssh -o StrictHostKeyChecking=no -t bisa@192.168.11.133 "cd ~/kmc_cav_docker && echo \" \" | sudo -S docker compose build && echo \" \" | sudo -S docker compose up; exec bash"
'

echo ""
echo "=========================================="
echo "4개 터미널이 자동으로 실행되었습니다."
echo "비밀번호 입력 없이 자동 실행됩니다."
echo "=========================================="
