# Control of Wheeled Mobile Robots: An Experimental Overview

Alessandro De Luca, Giuseppe Oriolo, Marilena Vendittelli

Dipartimento di Informatica e Sistemistica, Universit`a degli Studi di Roma “La Sapienza”, Italy

## Model

Pfaffian nonholonomic systems

$$ A(q)\dot{q} = 0 $$

kinematic model

$$ \dot{q} = G(q) w $$

## 2 Wheeel Vehicle

state variable

$$
\newcommand{\vvec}[1]{\left[\begin{array}{c}#1\end{array}\right]}
q := \vvec{x \\ y \\ \theta}^T 
$$

dynamics

$$
\def\R{\mathbb{R}}
\newcommand{\vvec}[1]{\left[\begin{array}{c}#1\end{array}\right]}
\dot{q} = \vvec{\dot{x} \\ \dot{y} \\\dot{\theta}} = \vvec{\cos\theta \\ \sin\theta \\ 0} u_v + \vvec{0\\0\\1} u_\omega \\
\dot{q} = g_v(q) u_v + g_\omega(q) u_\omega
$$

Lie bracket of inputs

$$
\newcommand{\partfrac}[2]{\frac{\partial #1}{\partial #2}}
\newcommand{\vvec}[1]{\left[\begin{array}{c}#1\end{array}\right]}
[g_v, g_\omega] = \partfrac{g_\omega}{q}g_v - \partfrac{g_v}{q}g_\omega
= \vvec{\sin\theta \\ -\cos\theta \\ 0}
$$

$$
\newcommand{\vvec}[1]{\left[\begin{array}{c}#1\end{array}\right]}
q_d(t) := \vvec{x_d(t) \\ y_d(t) \\ \theta_d(t)}
$$

$$
\tilde{q} := q - q_d \\
\tilde{v} := v - v_d \\
\tilde{\omega} := \omega - \omega_d
$$

$$
\newcommand{\vvec}[1]{\left[\begin{array}{c}#1\end{array}\right]}
\newcommand{\vmat}[2]{\left[\begin{array}{#1}#2\end{array}\right]}
\dot{\tilde{q}} = \vmat{ccc}{
    0 & 0 & -v_d\sin\theta_d \\
    0 & 0 & v_d\cos\theta_d \\
    0 & 0 & 0
} \tilde{q} +
\vmat{cc}{\cos\theta_d & 0 \\ \sin\theta_d & 0 \\ 0 & 1}
\vvec{\tilde{u_v}\\\tilde{u_\omega}}
= A(t) \tilde{q} + B(t)
\vvec{\tilde{u_v}\\\tilde{u_\omega}}
\tag{3.5}
$$

$$
\newcommand{\vmat}[2]{\left[\begin{array}{#1}#2\end{array}\right]}
\tilde{q}_R = 
\vmat{ccc}{
    \cos\theta_d & \sin\theta_d & 0 \\
    -\sin\theta_d & \cos\theta_d & 0 \\
    0 & 0 & 1
}
\tilde{q}
$$

## 5.1 Feedforward command design

$$
v_d(t) = \pm \sqrt{\dot{x}^2_d(t) + \dot{y}^2_d}
\\
\omega_d(t) = \frac{\ddot{y}_d(t)\dot{x}_d(t)-\ddot{x}_d(t)\dot{y}_d(t)}{\dot{x}_d^2(t) + \dot{y}_d^2(t)}
$$

$$
\newcommand{\vvec}[1]{\left[\begin{array}{c}#1\end{array}\right]}
\newcommand{\vmat}[2]{\left[\begin{array}{#1}#2\end{array}\right]}
\vvec{e_1\\e_2\\e_3}
:=
\vmat{ccc}{\cos\theta&\sin\theta&0\\-\sin\theta&\cos\theta&0\\0&0&1}
\vvec{x_d-x\\y_d-y\\\theta_d-\theta}
$$

$$

v &= v_d\cos e_3 - u_v
\\
\omega &= \omega_d - u_2
$$
