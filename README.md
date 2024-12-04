# final_project

# README

master_node.hpp에 함수, bool 변수 추가

```cpp
bool playYawFlag = false;

void ctlDxlYaw(float target_yaw);
```

master_node.cpp

함수 정의 추가

## 사용법

```cpp
ctlDxlYaw(원하는 yaw값);
```

flag처리 다 해놨으니 함수 불러주기만 하면 됌, 그럼 runDxl불러와서 회전까지 처리되는 함수임
