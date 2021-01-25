---
documentclass: scrreprt
fontsize: 14pt
header-includes: |
  \makeatletter
  \def\fps@figure{h}
  \makeatother
---
# IACV homework

## Geometry

### Functions and class

To simplify the developing of the required result I chose to write some classes and functions.

#### Class

- `HX`
  
    Represents a homogeneous vector. The multiplication between two `HW` instances is interpreted as a cross product, moreover provides some function to draw as line or point.

- `Seg`
  
    Represents a line segment. It provides method `line` to retrieve the associated line.
  
- `SegGroup`

    Represents a group of line segments. The method `find_vanish_point` finds the intersection point of the lines as an optimization problem.

#### Functions

- `get_normalized_transformation`
    
    Given a set of homogeneous points returns a similar transformation to normalize them. 

### G1 - 2D reconstruction

#### Recovery of the affine properties

To recover the affine properties of the images we have to put back the line at infinity in the image to its canonical position $\begin{bmatrix}0&0&1\end{bmatrix}^T$.
In order to do it we need to compute the infinity line for the plane $\Pi$ in the image, on this line lie all the vanish point s given by the parallel lines of the plane $\Pi$. So, I choice to compute the vanish points for the facade on which there are the line segments 2, 3 and 5 (the vanish point of the facades 1, 4 and 6  would introduce too much uncertainty, because the lines lie on them are almost parallel).

So I selected some lines parallel to plane $\Pi$ on the facades 2, 3 and 5; for each plane I created an instance of `SegGroup` to group the parallel line segments. With the method `find_vanish_point` I retrieve the vanish points corresponding to the three lines groups.

![parallel lines](./output/parallel_lines.png){height=250px}

The `find_vanish_point` sets the problem to find the vanish point as a minimization one. We know that a point $p$ on a line $l$ solves the relation $l^Tp=0$, thus the lines are collected in a matrix $L=\begin{bmatrix}l_1 & l_2 & \dots\end{bmatrix}^T$ where the best approximation fot the vanish point is the point $v$ that minimize the relation $\lVert Lv \rVert$.
The point $v$ that minimize the error is found exploiting the *least squares solution of homogeneus equation*  as the last column of the matrix $V$ getting from the *singular value decomposition* of the matrix $L$.
*In order to reduce the error given by the `svd` function, the line coordinates are normalized rescaling them around zero.*

![vanish points](./output/vanish_points.png) 

Found the 3 vanish points I use them to find the infinity line in the image, which have to pass to all the three vanish points. Due to the noise the infinity line cannot satisfy the relation $v l_\infty = 0$ for all the three vanish points, so I set also the problem to find infinity line as a minimization one. In this case I grouped the vanish points in a matrix $V = \begin{bmatrix}v_2 & v_3 & v_5\end{bmatrix}^T$ looking for the line $l_\infty$ which minimize $\lVert V l_\infty \rVert$.
Also to solve this minimization problem I used the `svd` function after the data were been normalized to reduce errors.

![infinity line](./output/infinity_line.png)

The homography to restore the affinity property can be written as

$$
H_p = \begin{bmatrix}
        1 & 0 & 0 \\
        0 & 1 & 0 \\
          & l_\infty^T &   \\
\end{bmatrix}
$$

Apply the homography to the image we can restore the affine properties. 

![affine properties rectified](./output/affine_rectification.png){height=250px}

#### Recovery of the metric properties

In order to recover the metric properties we can exploit the line infinity conic $C_\infty^\ast$.
We have to put back this line conic to its canonical position, do to that we can exploit the relation $l^T C_\infty^\ast m = 0$ where $l$ and $m$ are two orthogonal lines.
The $C_\infty^\ast$ after an affine rectification has the  form

$$
C_\infty^\ast =\begin{bmatrix}
    KK^T & 0  \\
    0^T & 0 \\
\end{bmatrix} \qquad
KK^T = \begin{bmatrix}
s_1 & s_2  \\
s_2 & s_3 \\
\end{bmatrix} = S
$$

So the previously seen constraint can be written as $a^T s = 0$ where $a=\begin{bmatrix}l_1 m_1 & l_1 m_2 + l_2 m_1 & l_2 m_2\end{bmatrix}^T$ and $s=\begin{bmatrix}s_1 & s_2 & s_3\end{bmatrix}^T$.
$s$ can be found solving the system $A^T s = 0$ where $A$ is a matrix which columns are composed by the relation $a(l,m)$ for at least 2 couples of orthogonal lines, but because we have 3 couples of orthogonal lines in the plane $\Pi$ (1-2, 4-5, 5-6) it is useful set the problem to find $s$ as an optimization one. The problem to minimize $\lVert A^Ts \rVert$ can be solved exploiting the `svd` method.

Gotten $S$ and thus $C_\infty^\ast$ we can find the homography that put back this conic to its canonical position under the relation $\bar C_\infty^\ast = H_A C_\infty^\ast H_A^T$ where $\bar C_\infty^\ast$ is the canonical infinity line conic.
$H_A$ can be gotten exploiting SVD of $C_\infty^\ast$ (because of $C_\infty^\ast$ have last column and last row equal to zero is better to decompose $S$) as $H_A^{-1} = U*\sqrt{D} * V^T + diag(0, 0, 1)$.

*Affinity transformation may include a mirror effect, so I decide to remove this effect to re-orientate the image in the original orientation if this effect appears.*

![metric properties rectified](./output/metric_rectification.png){height=250px}

#### Fix a reference frame on $\Pi$

To better evaluate the points on plane $\Pi$ I decided to compute a further similar transformation to assign an arbitrary reference frame on $\Pi$.
I chose to put the reference frame at the intersection of the lines 1 and 2, with y-axis aligned with the line 2 and with z-axis headed to the sky;
first I applied a translation to define the new origin, then I calculated the rotation matrix to align the line $2$ to the y-axis, and as last step I rescaled the points to meter unit exploiting an approximate measure of the segment $5$ gotten from internet. 

![$\Pi$ on reference frame](./output/pi_reference_frame.svg){height=250px}

### G2 - Calibration

#### Vertical vanish point

I selected the vertical lines shown in the figure to find the vertical vanish point. As previously I put the vertical segments in an instance of `SegGroup` and I used the method `find_vanish_point` to find the intersection point of the segments' associated lines. As previously saw, the function `find_vanish_point` solve the problem to find intersection point as an optimization one, exploiting `svd` after the data normalization.

![vertical vanish point](./output/vertical_vanish.png){height=500px}

#### Calibration

Now, we have 4 vanish point and a metric rectification homography that we can use to compute the intrinsic parameters of the camera.
In particular, I used the technique of the conical fitting. This technique is based on set an optimization problem to find the conic $\omega = (KK^T)^{-1}$. Due to the assumption of skew equal to zero the conic can be written as

$$
\omega = \begin{bmatrix}
    w_1 & 0 & w_3 \\
    0 & w_2 & w_4 \\
    w_3 & w_4 & w_5
\end{bmatrix}
$$

it is invariant to scale then it has 4 dof. 

I set the first constraint exploiting the homography computed at the point **G1**: given a metric rectified homography ($H^{-1}$) we can find the projection of the two canonical circular points $\begin{bmatrix} 1 & \pm i & 0 \end{bmatrix}^T$ in the image, which intersect $\omega$ conic. With the inverse homography we can map the circular points in the image as $H \begin{bmatrix} 1 & \pm i & 0 \end{bmatrix}^T$ getting $h_1 \pm i h_2$ where $h_1$ and $h_2$ are the columns of $H$.
The conic have to satisfy the constraints $(h_1 \pm i h_2)^T \omega (h_1 \pm i h_2) = 0$ which can be rewritten as $h_1^T \omega h_2 = 0$ and $h_1^T \omega h_1 - h_2^T \omega h_2 = 0$ that guarantee 2 constraints.

The others two required constraints are given by ones based on orthogonality: given two vanish points $v, u$ corresponding to orthogonal lines these satisfy the equation $v^T \omega u = 0$. I tried this way used the couple of vanish points given by the plane 2, 3, 5 and vertical ones, but due to numerical error these introduce too uncertainty, so I decide to use the alternative constraint based on a vanish line $l$ and a vanish point $v$ corresponding to orthogonal plane and line $[l]_x \omega v = 0$, where $[l]_x$ is the $\mathbb{P}^2$ version of the Plücker matrix of $l$ such that

$$
[l]_x = \begin{bmatrix}
    0 & -l_3 & l_2 \\
    l_3 & 0 & -l_1 \\
    -l_2 & l_1 & 0
\end{bmatrix}
$$

This equation gives two other constraints.
For these, I chose the vertical vanish point and the infinity line found at the point **G1**.

Before compose the matrix for the `svd` the data were been normalized exploiting the function `get_normalized_transformation`.

The parameters of $\omega$ were stacked in a vector $w$ and the constraints were rewritten in the form $A w = 0$, the solution for $w$ was found exploiting the `svd` then recomposed in the shape of conic $\omega$, exploiting *Cholesky* algorithm and before the de-normalization the matrix $K$ was found.

$$
K = \begin{bmatrix}
       3067.6 &           0 &        2003 \\
            0 &      3066.1 &      1501.4 \\
            0 &           0 &           1
\end{bmatrix}
$$

