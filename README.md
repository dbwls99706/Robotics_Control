# Robot Control Term Projects

MATLAB 기반으로 구현한 5개의 로봇 제어 프로젝트입니다. 

---

## 구조
```bash
├── project1
│   ├── Free_Fall_3DOF_Simulation.m
│   └── three_link.m
│
├── project2
│   ├── parameter_estimation_2dof.m
│   └── two_link.m
│
├── project3
│   ├── jointspace.m
│   └── one_link.m
│
├── project4
│   ├── get_Coriollis.m
│   ├── get_Gravity.m
│   ├── get_Inertia.m
│   ├── get_Jacobian.m
│   ├── get_Kinematics.m
│   ├── main_project4.m
│   └── two_links.m
│
├── project5_1
│   ├── HT.m
│   ├── joint_3dof.m
│   ├── three_link.m
│   ├── three_link_G.m
│   └── three_link_I.m
│
└── project5_2
    ├── cartesian_3dof.m
    ├── HT.m
    ├── three_link.m
    ├── three_link_C.m
    ├── three_link_G.m
    ├── three_link_I.m
    ├── three_link_J.m
    └── three_link_K.m
```
---


## 프로젝트 요약

### 1. project1: 3-DOF 자유 낙하 시뮬레이션  
- **파일**: `Free_Fall_3DOF_Simulation.m`, `three_link.m`  
- **내용**: 라그랑지안 기반 동역학 모델로 3-DOF 링크 시스템을 무토크 상태에서 자유 낙하 시뮬레이션. `ode45` 통합 후 애니메이션으로 관절 각도 변화 시각화.
![freefall_3DOFonline-video-cutter com-ezgif com-video-to-gif-converter](https://github.com/user-attachments/assets/50790802-bdb7-454a-bb15-009caaac11c3)

### 2. project2: 2-DOF 파라미터 추정  
- **파일**: `parameter_estimation_2dof.m`, `two_link.m`  
- **내용**: 칼만 필터와 Error Minimization 기법을 사용해 2-DOF 로봇의 동역학 파라미터(\(\mathbf{M}, \mathbf{C}, \mathbf{G}\))를 추정하고 수렴 속도 비교.
![image](https://github.com/user-attachments/assets/88f66cc1-4d5c-4603-9893-7a1cdf6c8d23)
![image](https://github.com/user-attachments/assets/4747ca49-47af-4bc2-b9e5-b0c984f8454a)

### 3. project3: 1-DOF Joint-Space PID 제어  
- **파일**: `jointspace.m`, `one_link.m`  
- **내용**: 단일 링크를 대상으로 한 PID 제어기 설계. 목표 각도에 대한 스텝 응답을 그래프로 분석.
![image](https://github.com/user-attachments/assets/06c86c68-ffa0-416f-afad-1e11ef81328c)

### 4. project4: 2-DOF Cartesian-Space PID 제어  
- **파일**: `main_project4.m`, `get_*`, `two_links.m` 등  
- **내용**: 엔드이펙터가 주기적으로 원 궤적을 그리도록 Cartesian-Space에서 PID 제어. 자코비안, 관성·코리올리스·중력 보상 함수 분리 구현.
![image](https://github.com/user-attachments/assets/64a2abcb-129b-4455-b6c8-59f2714c99b1)

### 5. project5_1: 3-DOF Joint-Space PID CTM Controller  
- **파일**: `HT.m`, `joint_3dof.m`, `three_link*.m`  
- **내용**: 문제3-1을 3자유도로 확장. 각 관절 목표 각도(90°, 60°, 30°) 및 일정 속도(30°/s)로 제어, 수렴 및 오버슛 분석.
![image](https://github.com/user-attachments/assets/b07bcb31-81c9-4c13-b5f3-63b1b7c281f7)

### 6. project5_2: 3-DOF Cartesian-Space PID CTM Controller  
- **파일**: `cartesian_3dof.m`, `HT.m`, `three_link_*.m`  
- **내용**: 문제3-2를 3자유도로 확장. 엔드이펙터 경로 생성(원 궤적), 역기구학 및 PID 제어 적용.
![image](https://github.com/user-attachments/assets/ca25d815-be78-4b57-a289-37b5ace05cdf)

---
