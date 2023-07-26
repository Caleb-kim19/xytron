# xytron

ssh 접속방법
ssh pi@<라즈베리파이 ip주소> 
ssh 비밀번호는 xytron

자율주행 실행하는 법
현재 파일 위치를 /xycar_ws/src/ 로 이동 후 다음 명령어 실행
$ roslaunch <폴더이름> <런치파일이름> 즉
$ roslaunch oval_team1 hough_drive_c1.launch

파일 정리할 땐 
$ cm

주행 영상 저장하는 법
1. 카메라 영상 담을 공간 설정
$ rosbag record /usb_cam/image_raw/compressed  

2. 차량을 구동시키기 - 구동이 끝나면 자동으로 저장됨

3. 저장된 영상 데이터 재생하기 & avi 파일로 저장하기
터미널 4개를 띄우고 다음의 명령을 각각 순서대로 실행
$ roscore
$ rosrun image_view video_recorder image:="/usb_cam/image_raw" _filename:="my_video.avi" _image_transport:="compressed"
$ rosrun image_view image_view image:=/usb_cam/image_raw _image_transport:=compressed
$ rosbag play 저장된백업파일.bag


