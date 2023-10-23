
# node_gen

가상의 node DAG을 임의로 구성하기 위한 node generator 패키지

# 실행 예시

ros2 run node_gen variable_payload_sub -> subscriber

ros2 run node_gen payload_pub v1 t1 1000 0 1000 3000 10 -> publisher


# simple example (4 node in a way)

ros2 run node_gen payload_pub v1 t1 1000 0 1000 3000 10

ros2 run node_gen variable_payload_sub v2 t1 t2 0

ros2 run node_gen variable_payload_sub v3 t2 t3 0

ros2 run node_gen variable_payload_sub v4 t3 t4 0

# how to build

colcon build --packages-select node_gen


ㅁㄴㄴㅁㄹㄴㅇㄹㅇㅁㄴㅇㄻㄴㅇㅁㄴㄴㅁㅇㄻㄴㅇㄻㅇㄴㄹㅇㄴㅇㄹㄴㅁㅇㄻㄴㅇㄻㄴㅇㄻㅇㄴㄴㅇㄻㄴㅇㄻㄴㅇㄴㅁㅇㅁㅁㄴㅇㄻㄴㅁㅇㄴㄹㄴㅇㄹㅇㄹㅇㄴㅁㄴㅇㄻㄴㅇㄻㄴㅇㄻㄴㅇㄻㅇㄴㄻㄴㅇㅁㄴㅇㄻㅇㄴㄹㄹㄻㅁㄴㅇㄹㄴㅇㄹㄴㅁㄴㅇㄻㄴㅇㄻㄴㅇㄻㅁㄴㅇㄻㄴㅇㄻㄴㅇㄹㅇㄴㅇㄴㅁㅇㄹㅇㄻㅇㄴㄴㅁㅇㄻㄴㅁㄴㅇㄹㄴㅇㄻㅇㄴㅇㄻㅇㄴㅁㄴㅁㅇㄻㄴㅇㄻㄴㅇㄹㄴㅇㅇㄴㄴㅇㅁㄹㄴㅁㅇㄹㅇㅁㄴㄻㅇㄴㄴㅁㅇㄻㄴㅇㄻㄴㅇㄹㄹㅇㅁㄴㄹㄴㅇㄹㅇㄴ
