.. _rudder_model:

Rudder model
============

Frames and conventions
----------------------

The convention used here is NWU.

The drag and lift contributions are expressed respectfully collinear and perpendicular to the inflow velocity field :math:`V_{fR}`,
with the introduction of a frame :math:`R_f=(O_f,x_f,y_f,z_f)` defined by:

- :math:`O_f` is ont the rudder stock, the rudder rotation axis,
- :math:`z_f` is collinear to the rudder stock, pointing upwards,
- :math:`y_f = z_f \times \dfrac{-V_{fR}{|V_{fR}|}`, in order to get lift and drag forces in a plane perpendicular to the rudder axis,
- :math:`x_f = y_f \times z_f`, so that the frame is orthogonal and direct


.. _fig_rudder_frame:
.. figure:: ../_static/rudder_frame.png
    :align: center
    :alt: rudder_frame

    Rudder frame representation

The rudder attack angle :math:`\alpha_r` can be expressed using the rudder deflection angle :math:`\delta_r` and the
local drift angle :math:`\beta_r`.

Expression of the force
-----------------------

The generalized force is expressed in the :math:`R_f` frame :

.. math::
   F_{rudder} =
   \begin{bmatrix}
    D_{rudder} & 0 \\
    L_{rudder} & 0 \\
    0          & N_{rudder}
   \end{bmatrix}_{R_f}

with the drag, lift and torque components

.. math::
    \begin{cases}
        D_{rudder} &=& \dfrac{1}{2} \rho C_d(\alpha_r) A_r V_{fR}^2\\
        L_{rudder} &=& \dfrac{1}{2} \rho C_l(\alpha_r) A_r V_{fR}^2\\
        N_{rudder} &=& \dfrac{1}{2} \rho C_n(\alpha_r) A_r c V_{fR}^2\\
    \end{cases}

where :math:`\rho` is the water density, :math:`A_r` is the projected lateral area and :math:`c` is the rudder chord.

Hull/rudder interactions
------------------------

As for the propeller, the rudder longitudinal velocity, relatively to the inflow velocity, can be corrected with a rudder wake fraction

.. math::
    u_{RA} = u_{R0}(1-\omega_r)

where :math:`u_{R0}` is the vessel longitudinal velocity, relatively to the surrounding flow, at the rudder position.

.. math::
    V_{fRA} = - V_{RA} = - u_{RA} x_v - v_{RA} yv

The rudder wake fraction can be expressed as a function of the vessel sidewash angle :math:`\beta_{R0} = atan2 \left(\dfrac{v_{R0}}{u_{R0}} \right)`, at the rudder position:

.. math::
    \omega_r = \omega_{r0} e^{-K_1 \beta_{R0}^2}

.. warning::
    Développer le hull straightening


References
----------
