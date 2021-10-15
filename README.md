# SlamPractice

Here is a simple script to convert the equation into html: https://jsfiddle.net/8ndx694g/    
or use \<img src="https://render.githubusercontent.com/render/math?math=\color{white}{e^{i +\pi} =x+1}"\>

## Practice 1 simple cmake practice
Create a Project named "cmake_practice" for a chance to writting CMakeList.txt file and getting a good orgnization of Project with ~/project/include and ~/project/src folds

## 6. Non Linear Optimization Problem
### 6.1 Status Estimation
Typical SLAM equation  

<img src="https://render.githubusercontent.com/render/math?math=\color{white}{\big\{ \begin{matrix} \mathbf{x}_{k} = \mathcal{F}(\mathbf{x}_{k-1}, \mathbf{u}_{k}) %2B \mathbf{w}_{k} \\ \mathbf{z}_{k,j} = \mathcal{H}(\mathbf{y}_{j},\mathbf{x}_{k}) %2B \mathbf{v}_{k,j}\end{matrix}}">    

where <img src="https://render.githubusercontent.com/render/math?math=\color{white}{\mathbf{x}_{k}}"> is the camera's pose at the time <img src="https://render.githubusercontent.com/render/math?math=\color{white}{k}">, <img src="https://render.githubusercontent.com/render/math?math=\color{white}{\mathbf{u}_{k}}"> is the information of senser at the time <img src="https://render.githubusercontent.com/render/math?math=\color{white}{k}">. <img src="https://render.githubusercontent.com/render/math?math=\color{white}{\mathbf{z}_{k,j}}"> is the pixel coordinate perceived by camera at the time <img src="https://render.githubusercontent.com/render/math?math=\color{white}{k}"> for landmark <img src="https://render.githubusercontent.com/render/math?math=\color{white}{\mathbf{y}_{j}}">. Especaily, <img src="https://render.githubusercontent.com/render/math?math=\color{white}{\mathcal{F}}"> and <img src="https://render.githubusercontent.com/render/math?math=\color{white}{\mathcal{H}}"> can be any kind of functions.

According to the camera's thoery, we have    
<img src="https://render.githubusercontent.com/render/math?math=\color{white}{s\mathbf{z}_{k,j}=\mathbf{K}(\mathbf{R}_{k}\mathbf{y}_{j}%2B\mathbf{t}_{k})}">
where, <img src="https://render.githubusercontent.com/render/math?math=\color{white}{\mathbf{K}}"> is the camera's internal parameters. <img src="https://render.githubusercontent.com/render/math?math=\color{white}{s}"> is the distance between each two pixels. <img src="https://render.githubusercontent.com/render/math?math=\color{white}{\mathbf{R}_{k}}"> is the rotation matrix of the camera at the time <img src="https://render.githubusercontent.com/render/math?math=\color{white}{k}">, <img src="https://render.githubusercontent.com/render/math?math=\color{white}{\mathbf{y}_{j}}"> is the <img src="https://render.githubusercontent.com/render/math?math=\color{white}{j_{th}}"> landmark. Furthermore, <img src="https://render.githubusercontent.com/render/math?math=\color{white}{\mathbf{t}_{k}}"> is the camera's translation at the time <img src="https://render.githubusercontent.com/render/math?math=\color{white}{k}">.

When <img src="https://render.githubusercontent.com/render/math?math=\color{white}{\mathbf{w}_{k}}"> and <img src="https://render.githubusercontent.com/render/math?math=\color{white}{\mathbf{v}_{k,j}}"> are considered as Gaussain Noise in this scenario. For getting the minimum error for estimation of <img src="https://render.githubusercontent.com/render/math?math=\color{white}{\hat{\mathbf{x}}_{k}}"> and <img src="https://render.githubusercontent.com/render/math?math=\color{white}{\hat{\mathbf{z}}_{j,k}}">, the method of least squares is employed to get the optimized solutions. With the knowledge of multidimensional Gaussain distribution    

<img src="https://render.githubusercontent.com/render/math?math=\color{white}{p(\cdot)=\frac{1}{(2\pi)^{N}det(\Sigma)}exp\big(-\frac{1}{2}(\mathbf{\cdot}-\mathbf{\mu})^{T}\Sigma^{-1}(\mathbf{\cdot}-\mathbf{\mu})\big)}">

For fomular <img src="https://render.githubusercontent.com/render/math?math=\color{white}{\mathbf{x}_{k} = f(\mathbf{x}_{k-1}, \mathbf{u}_{k}) %2B \mathbf{w}_{k}}">, <img src="https://render.githubusercontent.com/render/math?math=\color{white}{\cdot}"> is <img src="https://render.githubusercontent.com/render/math?math=\color{white}{\mathbf{w}_{k}}">, and <img src="https://render.githubusercontent.com/render/math?math=\color{white}{\mathbf{\mu}_{k}}"> is assumed to be <img src="https://render.githubusercontent.com/render/math?math=\color{white}{0}">. <img src="https://render.githubusercontent.com/render/math?math=\color{white}{\mathbf{\Sigma}_{k}}"> is assumed to be <img src="https://render.githubusercontent.com/render/math?math=\color{white}{\mathbf{O}_{k}}">.

As the same thoery, for formular <img src="https://render.githubusercontent.com/render/math?math=\color{white}{\mathbf{z}_{k,j} = h(\mathbf{y}_{j},\mathbf{x}_{k}) %2B \mathbf{v}_{k,j}}">, <img src="https://render.githubusercontent.com/render/math?math=\color{white}{\cdot}"> is <img src="https://render.githubusercontent.com/render/math?math=\color{white}{\mathbf{v}_{k,j}}">, and <img src="https://render.githubusercontent.com/render/math?math=\color{white}{\mathbf{\mu}_{k}}"> is assumed to be <img src="https://render.githubusercontent.com/render/math?math=\color{white}{0}">. <img src="https://render.githubusercontent.com/render/math?math=\color{white}{\mathbf{\Sigma}_{k}}"> is assumed to be <img src="https://render.githubusercontent.com/render/math?math=\color{white}{\mathbf{Q}_{k}}">.


