# SlamPractice

Here is a simple script to convert the equation into html: https://jsfiddle.net/8ndx694g/    
or use \<img src="https://render.githubusercontent.com/render/math?math=\color{red}{e^{i +\pi} =x+1}"\>

## Practice 1 simple cmake practice
Create a Project named "cmake_practice" for a chance to writting CMakeList.txt file and getting a good orgnization of Project with ~/project/include and ~/project/src folds

## 6. Non Linear Optimization Problem
### 6.1 Status Estimation
#### 6.1.1 Basic Status Estimation Equation</center>

Typical SLAM equation  
<p align="center">
<img src="https://render.githubusercontent.com/render/math?math=\color{red}{\big\{ \begin{matrix} \mathbf{x}_{k} = \mathcal{F}(\mathbf{x}_{k-1}, \mathbf{u}_{k}) %2B \mathbf{w}_{k} \\ \mathbf{z}_{k,j} = \mathcal{H}(\mathbf{y}_{j},\mathbf{x}_{k}) %2B \mathbf{v}_{k,j}\end{matrix}}">    
</p>  

where <img src="https://render.githubusercontent.com/render/math?math=\color{red}{\mathbf{x}_{k}}"> is the camera's pose at the time <img src="https://render.githubusercontent.com/render/math?math=\color{red}{k}">, <img src="https://render.githubusercontent.com/render/math?math=\color{red}{\mathbf{u}_{k}}"> is the information of senser at the time <img src="https://render.githubusercontent.com/render/math?math=\color{red}{k}">. <img src="https://render.githubusercontent.com/render/math?math=\color{red}{\mathbf{z}_{k,j}}"> is the pixel coordinate perceived by camera at the time <img src="https://render.githubusercontent.com/render/math?math=\color{red}{k}"> for landmark <img src="https://render.githubusercontent.com/render/math?math=\color{red}{\mathbf{y}_{j}}">. Especaily, <img src="https://render.githubusercontent.com/render/math?math=\color{red}{\mathcal{F}}"> and <img src="https://render.githubusercontent.com/render/math?math=\color{red}{\mathcal{H}}"> can be any kind of functions.

According to the camera's thoery, we have  
<p align="center">
<img src="https://render.githubusercontent.com/render/math?math=\color{red}{s\mathbf{z}_{k,j}=\mathbf{K}(\mathbf{R}_{k}\mathbf{y}_{j}%2B\mathbf{t}_{k})}">
</p>   

where, <img src="https://render.githubusercontent.com/render/math?math=\color{red}{\mathbf{K}}"> is the camera's internal parameters. <img src="https://render.githubusercontent.com/render/math?math=\color{red}{s}"> is the distance between each two pixels. <img src="https://render.githubusercontent.com/render/math?math=\color{red}{\mathbf{R}_{k}}"> is the rotation matrix of the camera at the time <img src="https://render.githubusercontent.com/render/math?math=\color{red}{k}">, <img src="https://render.githubusercontent.com/render/math?math=\color{red}{\mathbf{y}_{j}}"> is the <img src="https://render.githubusercontent.com/render/math?math=\color{red}{j_{th}}"> landmark. Furthermore, <img src="https://render.githubusercontent.com/render/math?math=\color{red}{\mathbf{t}_{k}}"> is the camera's translation at the time <img src="https://render.githubusercontent.com/render/math?math=\color{red}{k}">.

When <img src="https://render.githubusercontent.com/render/math?math=\color{red}{\mathbf{w}_{k}}"> and <img src="https://render.githubusercontent.com/render/math?math=\color{red}{\mathbf{v}_{k,j}}"> are considered as Gaussain Noise in this scenario. For getting the minimum error for estimation of <img src="https://render.githubusercontent.com/render/math?math=\color{red}{\hat{\mathbf{x}}_{k}}"> and <img src="https://render.githubusercontent.com/render/math?math=\color{red}{\hat{\mathbf{z}}_{j,k}}">, the method of least squares is employed to get the optimized solutions. With the knowledge of multidimensional Gaussain distribution    
<p align="center">
<img src="https://render.githubusercontent.com/render/math?math=\color{red}{p(\cdot)=\frac{1}{(2\pi)^{N}det(\Sigma)}exp\big(-\frac{1}{2}(\mathbf{\cdot}-\mathbf{\mu})^{T}\Sigma^{-1}(\mathbf{\cdot}-\mathbf{\mu})\big)}">
</p>  

For fomular <img src="https://render.githubusercontent.com/render/math?math=\color{red}{\mathbf{x}_{k} = f(\mathbf{x}_{k-1}, \mathbf{u}_{k}) %2B \mathbf{w}_{k}}">, <img src="https://render.githubusercontent.com/render/math?math=\color{red}{\bullet}"> is <img src="https://render.githubusercontent.com/render/math?math=\color{red}{\mathbf{w}_{k}}">, and <img src="https://render.githubusercontent.com/render/math?math=\color{red}{\mathbf{\mu}_{k}}"> is assumed to be <img src="https://render.githubusercontent.com/render/math?math=\color{red}{0}">. <img src="https://render.githubusercontent.com/render/math?math=\color{red}{\mathbf{\Sigma}_{k}}"> is assumed to be <img src="https://render.githubusercontent.com/render/math?math=\color{red}{\mathbf{O}_{k}}">.

As the same thoery, for formular <img src="https://render.githubusercontent.com/render/math?math=\color{red}{\mathbf{z}_{k,j} = h(\mathbf{y}_{j},\mathbf{x}_{k}) %2B \mathbf{v}_{k,j}}">, <img src="https://render.githubusercontent.com/render/math?math=\color{red}{\bullet}"> is <img src="https://render.githubusercontent.com/render/math?math=\color{red}{\mathbf{v}_{k,j}}">, and <img src="https://render.githubusercontent.com/render/math?math=\color{red}{\mathbf{\mu}_{k}}"> is assumed to be <img src="https://render.githubusercontent.com/render/math?math=\color{red}{0}">. <img src="https://render.githubusercontent.com/render/math?math=\color{red}{\mathbf{\Sigma}_{k}}"> is assumed to be <img src="https://render.githubusercontent.com/render/math?math=\color{red}{\mathbf{Q}_{k}}">.

#### 6.1.2 Least Square Cost Fucntion

If we use the assumed value and prior assumption of multi-dimension Gaussain Distribution together, combining with the least square solution, the errors would be:   
<p align="center">
<img src="https://render.githubusercontent.com/render/math?math=\color{red}{\begin{matrix} e_{u,k} = x_{k} -f(x_{k-1}, u_{k}) \\ e_{z,j,k} = z_{k,j} -h(x_{k}, y_{j})\end{matrix}}">
</p>

And the cost function can be defined as:   
<p align="center">
<img src="https://render.githubusercontent.com/render/math?math=\color{red}{minJ(x,y)=\Sigma_{k}e^{T}_{k,u} O^{-1}_{k} e_{k,u} %2B \Sigma_{j}\Sigma_{k} e^{T}_{z, k, j} Q^{-1}_{k,j}e_{z,k,j}}">
</p>

### 6.2 Least Sqaure Optimization

When consider a general least square problem: 
<p align="center">
<img src="https://render.githubusercontent.com/render/math?math=\color{red}{\min_{x}F(x) = \frac{1}{2}\bigg\|f(x)\bigg\|^{2}_{2}}"> 
</p>

where, <img src="https://render.githubusercontent.com/render/math?math=\color{red}{x\in \mathbb{R}^{n}}">, <img src="https://render.githubusercontent.com/render/math?math=\color{red}{f}"> is any non-linear function:  
<p align="center">
<img src="https://render.githubusercontent.com/render/math?math=\color{red}{f(x): \mathbb{R}^{n}\rightarrow \mathbb{R}}">
</p>

We have four steps to get the numerical solution by iterations:   
1. Giving a initialized value <img src="https://render.githubusercontent.com/render/math?math=\color{red}{x_{0}}">
2. For the <img src="https://render.githubusercontent.com/render/math?math=\color{red}{k_{th}}"> iteration, seeking a small increament <img src="https://render.githubusercontent.com/render/math?math=\color{red}{\Delta x_{k}}"> to have <img src="https://render.githubusercontent.com/render/math?math=\color{red}{\Delta x_{k} = \argmin \bigg\|f(x_{k} %2B \Delta x_{k})\bigg\|^{2}_{2}}">
3. If <img src="https://render.githubusercontent.com/render/math?math=\color{red}{\Delta x_{k}}"> is samller than the criterion, then stop.
4. Else, let <img src="https://render.githubusercontent.com/render/math?math=\color{red}{x_{k+1}=x_{k}%2B\Delta x_{k}}">.

#### 6.2.1 First Order and Second Order Gradient Solution
If we have a Function <img src="https://render.githubusercontent.com/render/math?math=\color{red}{\mathbf{F}(x)}"> and study the monotonicity within the range <img src="https://render.githubusercontent.com/render/math?math=\color{red}{x_{k}%2B\Delta x_{k}}">, by using Taylor Series, we have  
<p align="center">
<img src="https://render.githubusercontent.com/render/math?math=\color{red}{F(x_k%2B\Delta x_k) \approx F(x_k) %2B \mathbf{J}(x_k)^{T}\Delta x_k %2B \frac{1}{2}\Delta x_k^T\mathbf{H}(x_k)\Delta x_k}">
</p>

where, <img src="https://render.githubusercontent.com/render/math?math=\color{red}{\mathbf{J}}"> is the Jacobian matrix and <img src="https://render.githubusercontent.com/render/math?math=\color{red}{\mathbf{H}}"> is the Hessian matrix.  
If we only keep the first order gradient coefficient in Taylor serires, we have  
<p align="center">
<img src="https://render.githubusercontent.com/render/math?math=\color{red}{\Delta x^{\ast} = arg min \bigg\( F(x_k) %2B \mathbf{J}(x_k)^{T}\Delta x_k\bigg\)}">
</p>









