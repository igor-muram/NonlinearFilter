# Nonlinear Filter

Implementation of a one-step iterative sequential filter to solve the problem of determining the trajectory of a quadcopter flying at the same altitude. The problem is solved in the Cartesian two-dimensional coordinate system.<br>

## Problem statement

![image](https://user-images.githubusercontent.com/54866075/126877977-876077fc-4357-49a4-b5e3-0b04c9f4a1b6.png)
![image](https://user-images.githubusercontent.com/54866075/126529171-96b73f02-8db4-420d-8617-8fcb0b4609b8.png)

## Task algorithm

![image](https://user-images.githubusercontent.com/54866075/126877993-88d30660-d29a-4524-953a-f878fad8cea3.png)
![image](https://user-images.githubusercontent.com/54866075/126529322-8219b6a7-d998-44db-8e13-bcb9f7002a8f.png)

## Problem model. Measurement model. Discretization

![image](https://user-images.githubusercontent.com/54866075/126878011-f157aeb3-6cc3-4d0a-8f49-b8bd0613b144.png)

## Testing

The results obtained with the one-step iterative sequential filter are compared with the results obtained with the extended Kalman filter. 

![image](https://user-images.githubusercontent.com/54866075/126878120-0ddfa97e-8487-43a3-9ba8-337ed5706c06.png)
![image](https://user-images.githubusercontent.com/54866075/126878131-7d62f729-84b4-41f9-98dd-13a802bcbec3.png)
![image](https://user-images.githubusercontent.com/54866075/126878138-1483a61e-85c3-49dc-b680-6b02eaa7f2d5.png)
![image](https://user-images.githubusercontent.com/54866075/126878148-bae2fe57-53f0-4d9e-80f2-eb0f57837dba.png)
![image](https://user-images.githubusercontent.com/54866075/126878158-994f301f-d440-49b0-90eb-59c21c6ecd90.png)
![image](https://user-images.githubusercontent.com/54866075/126878173-d43619c6-82b3-4daa-aa39-8a8a9353d54c.png)


## Advantages and disadvantages

![image](https://user-images.githubusercontent.com/54866075/126878186-9239675b-a37e-4886-8ab9-508a9f7e7f05.png)
