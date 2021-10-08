.. _validations:

Validations
===========

Non-regression
--------------

ACME was initiated after the RDX022 project, in which propulsion models were needed for the definition of a manoeuvrability
global model. At that time, Sutulo's models [Sutulo2015]_ were chosen, in order to simulate operations in the four quadrants.

Due to some inconsistencies, notably in the propeller-rudder interaction model, it was decided to change for Brix' model
for ACME's propeller-rudder interaction model. A hull-rudder interaction was also omitted in the RDX022 rudder model,
namely the correction on the lift force, via the introduction of the factor :math:`a_H` and the application point forward
shift by :math:`\Delta x_r`.

Turning circles (TC) and zig zag (ZZ) manoeuvres were simulated during the RDX022, for a tug, which manoeuvrability coefficients were
identified during the project. We present in this section the comparison between manoeuvres simulated with the RDX022 models
and ACME models, for the same vessel configuration.

Propeller and rudder models
---------------------------

No lift correction
++++++++++++++++++

We first consider propeller and rudder models, without the propeller/rudder interactions. ACME and RDX022 propeller and
rudder models being similar, except for the lift force correction added in ACME, results should be also similar.

Manoeuvres were simulated with and without the hull flow straightening effect, and results are similar for both cases, see :ref:`following figure <fig_propeller_rudder_no_lift_correction_TC>`.

.. _fig_propeller_rudder_no_lift_correction_TC:
.. figure:: ../_static/validations/propeller_rudder_no_lift_correction_TC.png
    :align: center
    :alt: propeller_rudder_no_lift_correction_TC

    Comparison between TC manoeuvres simulated with RDX022 and ACME propeller and rudder models, without the lift correction.





References
----------
