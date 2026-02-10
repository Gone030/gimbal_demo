# - header.frame_id: 이 메시지의 yaw/pitch/range/object_* 값들이 정의된 기준 프레임
# - 기준 프레임 축: +X 전방, +Y 좌측, +Z 상방 (right-handed)
# - 단위: yaw/pitch/object_yaw [rad], range [m]
# - yaw  : +Z 축 기준 회전 (위에서 내려다볼 때 CCW가 +)
# - pitch: +Y 축 기준 회전 (전방(+X)에서 상방(+Z)으로 올라가면 +)
# - object_yaw: 이미지 평면 내 물체 방향각(카메라 광축 기준 회전으로 취급, [rad])

std_msgs/Header header
float64 yaw
float64 pitch
float64 range
float64 object_yaw
