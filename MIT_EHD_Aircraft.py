# MIT-like electrohydrodynamic (EHD) aircraft in RCAIDE
#
# Modelled on the MIT solid-state aircraft (Xu et al., Nature 563, 2018): 5 m span, 2.45 kg, eight
# wire-to-NACA 0010 electrode units at 40.3 kV. Builds the aircraft, runs a static thrust sweep, flies
# 60 m of level flight at 5 m/s, saves plots to "MIT EHD Plots/", and writes an OpenVSP model with the
# electrodes drawn in.
#
# Needs the rcaide1.5.0EHDproj environment (RCAIDE_LEADS clone with the EHD thruster, installed
# with pip install -e, plus OpenVSP):
#   C:\Users\avite\anaconda3\envs\rcaide1.5.0EHDproj\python.exe MIT_EHD_Aircraft.py
#
# Values in the GUESS block are placeholders (spec 2.4.1 marks them unverified). Replace them with the
# paper's Extended Data before comparing against the real aircraft. The same aircraft also lives in the
# repo test RCAIDE_LEADS/VnV/Verification/network_electric/ehd_mit_aircraft_test.py; keep the two in
# step if you change values here.

import RCAIDE
print("RCAIDE version: " + RCAIDE.__version__)

# python imports
import numpy as np
import matplotlib.pyplot as plt
import os

# run from this script's folder so ./Airfoils/..., the plots and the .vsp3 export resolve no matter where it's launched from
os.chdir(os.path.dirname(os.path.abspath(__file__)))

# RCAIDE imports
from RCAIDE.Framework.Core                                      import Units, Data
from RCAIDE.Library.Plots                                       import *
from RCAIDE.Library.Methods.Powertrain                          import setup_operating_conditions
from RCAIDE.Library.Methods.Powertrain.Propulsors.EHD_Thruster  import compute_ehd_thruster_mass

# ----------------------------------------------------------------------------------------------------------------------
#  Aircraft data
# ----------------------------------------------------------------------------------------------------------------------
# Sourced (spec 2.4.1, R17, Part 2.1)
MIT = Data()
MIT.wingspan              = 5.0        # m, Xu et al. 2018
MIT.mass                  = 2.45       # kg, Barrett in The Conversation (secondary)
MIT.gap                   = 0.060      # m, Extended Data Fig. 1
MIT.emitter_diameter      = 0.2e-3     # m, 32 AWG stainless, Extended Data Fig. 1
MIT.electrode_span        = 3.0        # m, Extended Data Fig. 1
MIT.maximum_voltage       = 40.3e3     # V, HVPC regulated output in flight, Extended Data Fig. 2
MIT.hvpc_rated_power      = 600.       # W, He, Woolston & Perreault 2017 (R17)
MIT.flight_distance       = 60.        # m, ~60 m in 10-12 s (secondary)
MIT.flight_speed          = 5.0        # m/s, derived from the line above
MIT.reported_thrust       = 3.2        # N, secondary source, [UNVERIFIED]

# PLACEHOLDERS: guesses, not data
GUESS = Data()
GUESS.unit_rows                    = 4      # "two columns of four rows" (forum post, secondary)
GUESS.unit_columns                 = 2
GUESS.unit_spacing                 = 0.10   # m, the spec's own assumed S for the MIT-like test vector
GUESS.collector_chord              = 0.05   # m
GUESS.column_spacing               = 0.30   # m, streamwise distance between the two electrode columns
GUESS.electrode_top_z              = -0.15  # m, top electrode row below the wing
GUESS.wing_chord                   = 0.40   # m, gives 2.0 m^2 with the 5 m span
GUESS.collector_foam_density       = 30.    # kg/m^3
GUESS.collector_foil_areal_density = 0.05   # kg/m^2 (about 18 um aluminium foil)
GUESS.battery_series_cells         = 50     # puts the bus inside the 160-225 V HVPC input window

plane_name = 'MIT_EHD_aircraft'
plot_dir   = 'MIT EHD Plots/'

# ----------------------------------------------------------------------------------------------------------------------
#  Main
# ----------------------------------------------------------------------------------------------------------------------
def main():
    print('NOTE: chord, spacing, unit layout, wing, tail and battery are placeholders (GUESS block).')

    vehicle  = vehicle_setup()
    configs  = configs_setup(vehicle)
    analyses = analyses_setup(configs)
    mission  = mission_setup(analyses)
    missions = missions_setup(mission)

    sweep    = thrust_sweep(vehicle)
    results  = missions.base_mission.evaluate()
    print_flight_summary(vehicle, results)

    os.makedirs(plot_dir, exist_ok = True)
    plot_thrust_sweep(sweep)
    plot_mission(results)

    export_vsp_with_electrodes(vehicle, plane_name + '.vsp3')
    print('Wrote ' + os.path.abspath(plane_name + '.vsp3'))
    plt.show()
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Vehicle
# ----------------------------------------------------------------------------------------------------------------------
def vehicle_setup():
    vehicle                                   = RCAIDE.Vehicle()
    vehicle.tag                               = plane_name
    vehicle.mass_properties.max_takeoff       = MIT.mass
    vehicle.mass_properties.takeoff           = MIT.mass
    vehicle.mass_properties.operating_empty   = MIT.mass
    vehicle.mass_properties.center_of_gravity = [[0.12, 0., 0.]]
    vehicle.reference_area                    = MIT.wingspan * GUESS.wing_chord
    vehicle.flight_envelope.ultimate_load     = 3.0
    vehicle.flight_envelope.limit_load        = 2.0

    # ------------------------------------------------------------------
    #   Main wing
    # ------------------------------------------------------------------
    wing                         = RCAIDE.Library.Components.Wings.Main_Wing()
    wing.tag                     = 'main_wing'
    wing.areas.reference         = vehicle.reference_area
    wing.spans.projected         = MIT.wingspan
    wing.chords.root             = GUESS.wing_chord
    wing.chords.tip              = GUESS.wing_chord/2
    wing.chords.mean_aerodynamic = GUESS.wing_chord
    wing.taper                   = 1.0
    wing.aspect_ratio            = wing.spans.projected**2 / wing.areas.reference
    wing.thickness_to_chord      = 0.12
    wing.sweeps.quarter_chord    = 0.0
    wing.twists.root             = 0.0
    wing.twists.tip              = 0.0
    wing.origin                  = [[0.0, 0., 0.]]
    wing.aerodynamic_center      = [[0.25 * GUESS.wing_chord, 0., 0.]]
    wing.vertical                = False
    wing.xz_plane_symmetric      = True
    wing.dynamic_pressure_ratio  = 1.0
    airfoil                      = RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.tag                  = 'Clark_y'
    airfoil.coordinate_file      = os.path.join('Airfoils', 'Clark_y.txt')
    wing.append_airfoil(airfoil)
    vehicle.append_component(wing)

    #root
    root = RCAIDE.Library.Components.Wings.Segments.Segment()
    root.tag                                                = 'root' 
    root.percent_span_location                              = 0.0
    root.twist                                              = 0.0 * Units.deg
    root.root_chord_percent                                 = 1.0
    root.chords.root                                        = GUESS.wing_chord
    root.chords.tip                                         = GUESS.wing_chord
    root.thickness_to_chord                                 = 0.12
    root.dihedral_outboard                                  = 0 * Units.degrees
    root.sweeps.quarter_chord                               = 0 * Units.degrees
    airfoil                      = RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.tag                  = 'Clark_y'
    airfoil.coordinate_file      = os.path.join('Airfoils', 'Clark_y.txt')
    root.append_airfoil(airfoil)
    wing.append_segment(root) 
    
    #segment1
    segment1 = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment1.tag                                            = 'segment1' 
    segment1.percent_span_location                          = MIT.electrode_span / MIT.wingspan
    segment1.twist                                          = 0.0 * Units.deg
    segment1.root_chord_percent                             = 1
    segment1.thickness_to_chord                             = 0.12
    segment1.dihedral_outboard                              = 36 * Units.degrees
    segment1.sweeps.quarter_chord                           = 0 * Units.degrees
    airfoil                      = RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.tag                  = 'Clark_y'
    airfoil.coordinate_file      = os.path.join('Airfoils', 'Clark_y.txt')
    segment1.append_airfoil(airfoil)
    wing.append_segment(segment1) 
    
    #wingtip end
    tip = RCAIDE.Library.Components.Wings.Segments.Segment()
    tip.tag                                                 = 'tip' 
    tip.percent_span_location                               = 1.0
    tip.twist                                               = 0 * Units.deg
    tip.root_chord_percent                                  = wing.chords.tip/wing.chords.root
    tip.thickness_to_chord                                  = 0.12
    tip.dihedral_outboard                                   = 0 * Units.degrees
    tip.sweeps.quarter_chord                                = 12 * Units.degrees
    airfoil                      = RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.tag                  = 'Clark_y'
    airfoil.coordinate_file      = os.path.join('Airfoils', 'Clark_y.txt')
    tip.append_airfoil(airfoil)  
    wing.append_segment(tip)

    # ------------------------------------------------------------------
    #   Tail (placeholder sizes), mounted at the height of the lowest electrode row
    # ------------------------------------------------------------------
    tail_z = GUESS.electrode_top_z - (GUESS.unit_rows - 1) * GUESS.unit_spacing
    for tag, span, chord, x, vertical in (('horizontal_stabilizer', 1.2, 0.25, 1.6, False),
                                          ('vertical_stabilizer',   0.5, 0.25, 1.6, True)):
        tail                         = (RCAIDE.Library.Components.Wings.Vertical_Tail() if vertical
                                        else RCAIDE.Library.Components.Wings.Horizontal_Tail())
        tail.tag                     = tag
        tail.areas.reference         = span * chord
        tail.spans.projected         = span
        tail.chords.root             = chord
        tail.chords.tip              = chord
        tail.chords.mean_aerodynamic = chord
        tail.taper                   = 1.0
        tail.aspect_ratio            = span**2 / tail.areas.reference
        tail.thickness_to_chord      = 0.10
        tail.sweeps.quarter_chord    = 0.0
        tail.twists.root             = 0.0
        tail.twists.tip              = 0.0
        tail.origin                  = [[x, 0., tail_z]]
        tail.aerodynamic_center      = [[x + 0.25 * chord, 0., tail_z]]
        tail.vertical                = vertical
        tail.xz_plane_symmetric      = not vertical
        tail.dynamic_pressure_ratio  = 0.9
        vehicle.append_component(tail)

    # ------------------------------------------------------------------
    #   Energy network: battery -> bus -> HV converter -> EHD thruster
    # ------------------------------------------------------------------
    net = RCAIDE.Framework.Networks.Electric()
    bus = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus()
    bat = RCAIDE.Library.Components.Powertrain.Sources.Battery_Modules.Lithium_Ion_NMC()
    bat.electrical_configuration.series        = GUESS.battery_series_cells
    bat.electrical_configuration.parallel      = 1
    bat.geometric_configuration.normal_count   = GUESS.battery_series_cells
    bat.geometric_configuration.parallel_count = 1
    bus.battery_modules.append(bat)
    bus.battery_module_electric_configuration  = 'Series'
    bus.initialize_bus_properties()

    thruster                           = RCAIDE.Library.Components.Powertrain.Propulsors.EHD_Thruster()
    thruster.tag                       = 'ehd_thruster'
    thruster.origin                    = [[0.5 * GUESS.column_spacing + MIT.gap, 0.,
                                           GUESS.electrode_top_z - 0.5 * (GUESS.unit_rows - 1) * GUESS.unit_spacing]]
    array                              = RCAIDE.Library.Components.Powertrain.Converters.EHD_Electrode_Array()
    array.tag                          = 'ehd_electrode_array'
    array.gap                          = MIT.gap
    array.emitter_diameter             = MIT.emitter_diameter
    array.span                         = MIT.electrode_span
    array.number_of_units              = GUESS.unit_rows * GUESS.unit_columns
    array.unit_spacing                 = GUESS.unit_spacing
    array.collector_chord              = GUESS.collector_chord
    array.maximum_voltage              = MIT.maximum_voltage
    array.collector_foam_density       = GUESS.collector_foam_density
    array.collector_foil_areal_density = GUESS.collector_foil_areal_density
    hvpc                               = RCAIDE.Library.Components.Powertrain.Modulators.High_Voltage_Converter()
    hvpc.tag                           = 'hvpc'
    hvpc.rated_power                   = MIT.hvpc_rated_power
    hvpc.bus_voltage                   = bus.voltage
    thruster.electrode_array           = array
    thruster.high_voltage_converter    = hvpc
    compute_ehd_thruster_mass(thruster)
    net.propulsors.append(thruster)

    bus.assigned_propulsors = [[thruster.tag]]
    net.busses.append(bus)
    vehicle.append_energy_network(net)
    return vehicle

# ----------------------------------------------------------------------------------------------------------------------
#  Configurations
# ----------------------------------------------------------------------------------------------------------------------
def configs_setup(vehicle):
    configs         = RCAIDE.Library.Components.Configs.Config.Container()
    base_config     = RCAIDE.Library.Components.Configs.Config(vehicle)
    base_config.tag = 'base'
    configs.append(base_config)
    return configs

# ----------------------------------------------------------------------------------------------------------------------
#  Analyses
# ----------------------------------------------------------------------------------------------------------------------
def base_analysis(vehicle):
    analyses         = RCAIDE.Framework.Analyses.Vehicle()
    analyses.vehicle = vehicle

    # weight buildups don't know the EHD thruster, so the vehicle mass above is kept as set
    weights = RCAIDE.Framework.Analyses.Weights.Electric_Drone()
    weights.settings.run_weights_analysis = False
    analyses.append(weights)

    geometry = RCAIDE.Framework.Analyses.Geometry.Geometry()
    geometry.settings.overwrite_reference = False
    analyses.append(geometry)

    aerodynamics = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method()
    aerodynamics.settings.number_of_spanwise_vortices  = 10
    aerodynamics.settings.number_of_chordwise_vortices = 4
    analyses.append(aerodynamics)

    analyses.append(RCAIDE.Framework.Analyses.Energy.Energy())
    analyses.append(RCAIDE.Framework.Analyses.Planets.Earth())
    analyses.append(RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976())
    return analyses

def analyses_setup(configs):
    analyses = RCAIDE.Framework.Analyses.Analysis.Container()
    for tag, config in configs.items():
        analyses[tag] = base_analysis(config)
    return analyses

# ----------------------------------------------------------------------------------------------------------------------
#  Mission: 60 m of level flight at 5 m/s, 2 m above the ground
# ----------------------------------------------------------------------------------------------------------------------
def mission_setup(analyses):
    mission      = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag  = 'base_mission'
    Segments     = RCAIDE.Framework.Mission.Segments
    base_segment = Segments.Segment()
    base_segment.state.numerics.number_of_control_points = 10

    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = 'level_flight'
    segment.analyses.extend(analyses.base)
    segment.altitude                         = 2.0 * Units.m
    segment.air_speed                        = MIT.flight_speed * Units['m/s']
    segment.distance                         = MIT.flight_distance * Units.m
    segment.initial_battery_state_of_charge  = 1.0
    segment.flight_dynamics.force_x          = True
    segment.flight_dynamics.force_z          = True
    segment.assigned_control_variables.throttle.active                  = True
    segment.assigned_control_variables.throttle.assigned_propulsors     = [['ehd_thruster']]
    segment.assigned_control_variables.throttle.initial_guess_values    = [[0.3]]
    segment.assigned_control_variables.pitch_angle.active               = True
    segment.assigned_control_variables.pitch_angle.initial_guess_values = [[3.0 * Units.degree]]
    mission.append_segment(segment)
    return mission

def missions_setup(mission):
    missions = RCAIDE.Framework.Mission.Missions()
    missions.append(mission)
    return missions

# ----------------------------------------------------------------------------------------------------------------------
#  Static thrust sweep (sea level, 25 degC so the Peek relative air density is 1)
# ----------------------------------------------------------------------------------------------------------------------
def thrust_sweep(vehicle):
    thruster = vehicle.networks.electric.propulsors['ehd_thruster']
    throttle = np.linspace(0., 1., 21)
    state    = setup_operating_conditions(thruster, velocity_range = 0. * throttle, altitude = 0., temperature_deviation = 10.)
    state.conditions.energy.propulsors[thruster.tag].throttle[:,0] = throttle
    thruster.compute_performance(state)
    p   = state.conditions.energy.propulsors[thruster.tag]
    arr = state.conditions.energy.converters[thruster.electrode_array.tag]

    sweep          = Data()
    sweep.throttle = throttle
    sweep.voltage  = arr.applied_voltage[:,0]
    sweep.thrust   = p.net_thrust[:,0]
    sweep.power    = p.bus_power[:,0]
    sweep.T_over_P = p.thrust_to_power_ratio[:,0]

    print('\nStatic thrust sweep (sea level, 25 degC)')
    print('  throttle   V_a [kV]   T_net [N]   P_bus [W]')
    for i in range(0, len(throttle), 2):
        print('  %8.1f   %8.1f   %9.2f   %9.0f' % (throttle[i], sweep.voltage[i] / 1e3, sweep.thrust[i], sweep.power[i]))
    return sweep

def print_flight_summary(vehicle, results):
    seg = results.segments.level_flight
    c   = seg.conditions
    bus = list(vehicle.networks.electric.busses.values())[0]
    b   = c.energy.busses[bus.tag]
    p   = c.energy.propulsors['ehd_thruster']
    arr = c.energy.converters['ehd_electrode_array']
    print('\nLevel flight at %.1f m/s over %.0f m (converged: %s)' % (MIT.flight_speed, MIT.flight_distance, seg.converged))
    print('  net thrust    %.2f N   (reported for the real aircraft: %.1f N, secondary source)' % (p.net_thrust[0,0], MIT.reported_thrust))
    print('  throttle      %.3f' % p.throttle[0,0])
    print('  voltage       %.1f kV' % (arr.applied_voltage[0,0] / 1e3))
    print('  bus power     %.0f W' % b.power_draw[0,0])
    print('  SOC           %.4f -> %.4f' % (b.state_of_charge[0,0], b.state_of_charge[-1,0]))
    print('  thruster mass %.3f kg (wire, collectors, HVPC)' % vehicle.networks.electric.propulsors['ehd_thruster'].mass_properties.mass)
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Plots
# ----------------------------------------------------------------------------------------------------------------------
def tidy_and_save_figure(fig, filename, legend_title = 'Flight Segment'):
    """Move RCAIDE's segment legend beside the plots, rescale every axis to show all of its data, and save"""
    fig.set_size_inches(16, 9)

    axis_labels = set()
    for axis in fig.axes:
        if axis.get_legend() is not None:
            axis_labels.update(text.get_text() for text in axis.get_legend().get_texts())

    for leg in list(fig.legends):
        entries = [(handle, text.get_text()) for handle, text in zip(leg.legend_handles, leg.get_texts())
                   if text.get_text() not in axis_labels]
        leg.remove()
        fig.legend([e[0] for e in entries], [e[1] for e in entries], title = legend_title,
                   loc = 'center left', bbox_to_anchor = (0.81, 0.5), ncol = 1)

    for axis in fig.axes:
        axis.margins(y = 0.08)
        axis.relim()
        axis.autoscale(enable = True)
        if axis.get_legend() is not None:
            axis.get_legend().set_loc('best')
            axis.get_legend().get_frame().set_alpha(0.85)

    fig.tight_layout(rect = (0, 0, 0.80, 0.96))
    fig.savefig(filename + '.png', dpi = 150)
    return

def plot_thrust_sweep(sweep):
    fig, axes = plt.subplots(1, 3, figsize = (16, 5))
    kV = sweep.voltage / 1e3
    axes[0].plot(kV, sweep.thrust, 'o-')
    axes[0].axhline(MIT.reported_thrust, color = 'gray', ls = '--', label = 'reported 3.2 N (secondary)')
    axes[0].set_xlabel('Electrode voltage [kV]'); axes[0].set_ylabel('Static net thrust [N]'); axes[0].legend()
    axes[1].plot(kV, sweep.power, 'o-')
    axes[1].axhline(MIT.hvpc_rated_power / 0.85, color = 'red', ls = '--', label = '600 W converter limit (%.0f W from battery)' % (MIT.hvpc_rated_power / 0.85))
    axes[1].set_xlabel('Electrode voltage [kV]'); axes[1].set_ylabel('Battery-side power [W]'); axes[1].legend()
    axes[2].plot(kV[sweep.power > 0], 1e3 * sweep.T_over_P[sweep.power > 0], 'o-')
    axes[2].set_xlabel('Electrode voltage [kV]'); axes[2].set_ylabel('Electrical T/P [N/kW]')
    for axis in axes:
        axis.grid(True, alpha = 0.3)
    fig.suptitle('MIT-like EHD thruster: static sweep (placeholder geometry, uncalibrated 1-D model)')
    fig.tight_layout()
    fig.savefig(plot_dir + 'EHD_Thrust_Sweep.png', dpi = 150)
    return

def plot_mission(results):
    fig = plot_flight_conditions(results)
    tidy_and_save_figure(fig, plot_dir + 'Flight_Conditions')

    fig = plot_aerodynamic_forces(results)
    tidy_and_save_figure(fig, plot_dir + 'Aerodynamic_Forces')

    fig = plot_aerodynamic_coefficients(results)
    tidy_and_save_figure(fig, plot_dir + 'Aerodynamic_Coefficients')

    fig = plot_drag_components(results)
    tidy_and_save_figure(fig, plot_dir + 'Drag_Components', legend_title = 'Drag Component')

    fig = plot_propulsor_throttles(results)
    tidy_and_save_figure(fig, plot_dir + 'Throttle')

    fig = plot_battery_module_conditions(results)
    tidy_and_save_figure(fig, plot_dir + 'Battery_Module_Conditions')
    return

# ----------------------------------------------------------------------------------------------------------------------
#  OpenVSP export with the electrodes drawn in
# ----------------------------------------------------------------------------------------------------------------------
def export_vsp_with_electrodes(vehicle, filename, rows = GUESS.unit_rows, columns = GUESS.unit_columns,
                               column_spacing = GUESS.column_spacing, top_z = GUESS.electrode_top_z):
    """
    Writes the airframe with RCAIDE's OpenVSP exporter, then adds every emitter wire and NACA 0010 collector
    as a display-only OpenVSP wing. The electrodes go only into the .vsp3 file, never into the RCAIDE
    vehicle, because their drag is already inside the EHD propulsor.
    """
    import openvsp as vsp
    from RCAIDE.Framework.External_Interfaces.OpenVSP import export_vsp_vehicle

    # verbose must stay True: export_vsp_vehicle only writes wings inside its `if verbose:` block (RCAIDE bug)
    export_vsp_vehicle(vehicle, vehicle.tag, verbose = True, write_file = False)
    array = vehicle.networks.electric.propulsors['ehd_thruster'].electrode_array
    if rows * columns != array.number_of_units:
        raise ValueError('rows x columns must equal the number of EHD units')

    def add_strip(name, chord, x, z, shape):
        g = vsp.AddGeom('WING')
        vsp.SetGeomName(g, name)
        vsp.SetDriverGroup(g, 1, vsp.SPAN_WSECT_DRIVER, vsp.ROOTC_WSECT_DRIVER, vsp.TIPC_WSECT_DRIVER)
        vsp.SetParmVal(g, 'Span', 'XSec_1', array.span / 2)
        vsp.SetParmVal(g, 'Root_Chord', 'XSec_1', chord)
        vsp.SetParmVal(g, 'Tip_Chord', 'XSec_1', chord)
        vsp.SetParmVal(g, 'Sweep', 'XSec_1', 0.0)
        vsp.SetParmVal(g, 'Dihedral', 'XSec_1', 0.0)
        vsp.SetParmVal(g, 'X_Rel_Location', 'XForm', x)
        vsp.SetParmVal(g, 'Z_Rel_Location', 'XForm', z)
        surf = vsp.GetXSecSurf(g, 0)
        for i in range(vsp.GetNumXSec(surf)):
            vsp.ChangeXSecShape(surf, i, shape)
            xsec = vsp.GetXSec(surf, i)
            if shape == vsp.XS_FOUR_SERIES:
                vsp.SetParmVal(vsp.GetXSecParm(xsec, 'ThickChord'), 0.10)
                vsp.SetParmVal(vsp.GetXSecParm(xsec, 'Camber'), 0.0)
            else:
                vsp.SetParmVal(vsp.GetXSecParm(xsec, 'Circle_Diameter'), chord)
        vsp.Update()

    D_w = array.emitter_diameter
    for j in range(columns):
        x_wire = j * column_spacing
        for i in range(rows):
            z = top_z - i * array.unit_spacing
            add_strip('ehd_wire_c%d_r%d' % (j + 1, i + 1),      D_w, x_wire - D_w / 2, z, vsp.XS_CIRCLE)
            add_strip('ehd_collector_c%d_r%d' % (j + 1, i + 1), array.collector_chord, x_wire + array.gap, z, vsp.XS_FOUR_SERIES)
    vsp.WriteVSPFile(filename)
    return filename


if __name__ == '__main__':
    main()
