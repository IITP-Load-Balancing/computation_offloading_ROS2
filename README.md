# computation_offloading_ROS2 
ROS2 기반 자율주행 application의 로드밸런싱을 위한 computation offloading 기술 개발

## node_gen
가상의 node DAG을 임의로 구성하기 위한 node generator 패키지입니다.

## app-level_offloading_ros2
application level에서 코드를 수정하여, local computing node와 server computing node의 computation reuslt를 취사선택하는 패키지입니다.

## PriorityBasedROS2
"미래 모빌리티를 위한 우선 순위 기반 ROS2 통신 프레임워크: 지연 최소화 및 네트워크 처리량 최적화" 논문에서 제공한 네 가지 구성요소에 대한 구현 코드입니다. 
