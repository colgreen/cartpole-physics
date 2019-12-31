# cartpole-physics

C# source code for simulation of the cart and pole balancing task.

For details of this task, including derivations of the equations of motion, see the following technical research paper:

[Equations of Motion for the Cart and Pole Control Task](https://sharpneat.sourceforge.io/research/cart-pole/cart-pole-equations.html)

The source code in this reposistory can be considered to be supplementary material for that paper.


## Repository Overview

This repository contains C# source code (src folder) for the cart-pole balancing task, and some R/ggplot2 scripts (r-ggplot2 folder) for creating plots for the above linked research paper.

## Source Code Overview

The source is written in C# and targets .NET Standard 2.1 and .NET Core 3.1. However, the physics/maths code should be easy to port to
any other language.

Visual Studio 2019 was used to author the code, and thus the source code is structured as a containing Visual Studio 'solution' (CartPolePhysics.sln),
with three projects within it. These are:


### CartPolePhysics
This project contains the maths/physics source code. The project targets .NET Standard 2.1 as this provides a MathF class that is used for
the code that operates with single precision floating point variables, most notably MathF.Sin() and Cos().

  * Double
    * CartSinglePoleEquations.cs
      * Contains the equations of motion for the cart-pole model, giving the horizontal acceleration of the cart and the angular acceleration 
      of the pole for a given model state and external force f (a horizontal force pushing the cart left or right).
    * CartSinglePolePhysics.cs
      * Contains an instance of CartSinglePoleEquations, and applies Euler's method to update the model state from one timestep to the next.
      This is also the base class for CartSinglePolePhysicsRK2 and CartSinglePolePhysicsRK4.
    * CartSinglePolePhysicsRK2.cs
      * Contains an instance of CartSinglePoleEquations, and applies the standard second-order Runge-Kutta method to update the model
      state from one timestep to the next.
    * CartSinglePolePhysicsRK4.cs
      * Contains an instance of CartSinglePoleEquations, and applies the classic fourth-order Runge-Kutta method to update the model
      state from one timestep to the next.

  * Single
    * The classes in this folder/namespace mirror exactly those of the 'Double' folder, but provide physics calculation based on
    single precision floating point maths.


### CartPoleConsole
A small console app that can be used to run a simulation and save a CSV fiel containing the model state at each timestep. These CSV files
can then be used to make plots, e.g. with R and ggplot2.


### CartPoleWinForms
A small Windows Forms app that targets .NET Core 3.1. The ability to run Winforms apps in .NET Core is a recent addition to that platform,
and this is only possible when the app is running on Microsoft Windows.

This app runs a simulation and shows a graph on screen of pole angle, cart position, and pole angular velocity over time.


