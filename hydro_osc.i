aper_init = 3e-4
period = 60 # Oscillation period in s
dt = 5
endTime = 600
injection_amp = 1e-4 # Injection amplitude in (m^3/s)
p_atm = 101325
file_name = hydro_osc

[Mesh]
  [fmg]
    type = FileMeshGenerator
    file = 'multi_dim_mesh.e'
  []
  [frac]
    type = LowerDBlockFromSidesetGenerator
    new_block_id = 1
    new_block_name = 'fracture'
    sidesets = '1'
    input = fmg
  []
[]

[GlobalParams]
  PorousFlowDictator = dictator
  gravity = '0 0 -9.81'
[]

[UserObjects]
  [steady_solution]
    type = SolutionUserObject
    mesh = './out_files/hydro_steady.e'
    system_variables = 'pp'
    timestep = LATEST
  []
[]

[Variables]
  [pp]
  []
[]

[AuxVariables]
  [aperture]
    family = MONOMIAL
    order = CONSTANT
    initial_condition = ${aper_init}
    block = 'fracture'
  []
  [frac_perm]
    family = MONOMIAL
    order = CONSTANT
    block = 'fracture'
  []
  [hydrostatic]
    family = LAGRANGE
    order = FIRST
  []
  [velocity_x]
    family = MONOMIAL
    order = CONSTANT
    block = 'fracture'
  []
  [velocity_y]
    family = MONOMIAL
    order = CONSTANT
    block = 'fracture'
  []
  [velocity_z]
    family = MONOMIAL
    order = CONSTANT
    block = 'fracture'
  []
[]

[AuxKernels]
  [frac_perm]
    type = ParsedAux
    variable = frac_perm
    coupled_variables = aperture
    expression = '(aperture * aperture * aperture)/12'
  []
  [velocity_x]
    type = PorousFlowDarcyVelocityComponentLowerDimensional
    variable = velocity_x
    component = x
    aperture = aperture
  []
  [velocity_y]
    type = PorousFlowDarcyVelocityComponentLowerDimensional
    variable = velocity_y
    component = y
    aperture = aperture
  []
  [velocity_z]
    type = PorousFlowDarcyVelocityComponentLowerDimensional
    variable = velocity_z
    component = z
    aperture = aperture
  []
[]

[BCs]
  #Hydraulic Boundaries
  [const_press]
    type = FunctionDirichletBC
    variable = pp
    boundary = 'x_min x_max y_min y_max'
    function = hydrostatic
  []
  [no_flow]
    type = NeumannBC
    boundary = 'z_min z_max'
    variable = pp
    value = 0
  []
[]

[ICs]
  [pp_init]
    type = SolutionIC
    solution_uo = steady_solution
    variable = pp
    from_variable = 'pp'
  []
[]

[Functions]
  # Periodic pumping signal
  [mass_flux_fxn]
    type = ParsedFunction
    expression = 'if(t<0, 0, (Q_0*(1-exp(-t/P))*sin((t+(ph_off*P))*((2*pi)/P)))*3.5)'
    symbol_names = 'Q_0 P ph_off b'
    symbol_values = '${injection_amp} ${period} 0.5 ${aper_init}' 
  []
  # Hydrostatic pressure gradient
  [hydrostatic]
    type = ParsedFunction
    expression = 'p_atm + (rho_f*g*(z_max-z))'
    symbol_names = 'p_atm rho_f g z_max'
    symbol_values = '${p_atm} 998.23 9.81 225'
  []
  [mass_bal_pct]
    type = ParsedFunction
    expression = '((in - out)/in) * 100'
    symbol_names = 'in out'
    symbol_values = 'fluid_in fluid_out'
  []
[]


[PorousFlowBasicTHM]
  coupling_type = Hydro
  porepressure = pp
  fp = water
  gravity = '0 0 -9.81'
  use_displaced_mesh = false
  multiply_by_density = false
[]

[FluidProperties]
  [water]
    type = SimpleFluidProperties
    bulk_modulus = 2e9
    density0 = 998.23
    viscosity = 1e-3
  []
[]

[Materials]
 [frac_aper]
    type = PorousFlowPorosityConst
    porosity = aperture
    block = 'fracture'
  []
  [rock_porosity]
    type = PorousFlowPorosityConst
    porosity = 0.02
    block = 'rock'
  []
  [frac_perm]
    type = PorousFlowPermeabilityConstFromVar
    perm_xx = frac_perm
    perm_yy = frac_perm
    perm_zz = frac_perm
    block = 'fracture'
  []
  [rock_perm]
    type = PorousFlowPermeabilityConst
    permeability = '1e-22 0 0   0 1e-22 0   0 0 1e-22'
    block = 'rock'
  []
  [biot_mod]
    type = PorousFlowConstantBiotModulus
    biot_coefficient = 1
    solid_bulk_compliance = 3e-11
  []
[]

# Hydraulic injection kernel
[DiracKernels]
  # Periodic Flux
  [mass_flux_source]
    type = PorousFlowPointSourceFromPostprocessor
    mass_flux = mass_flux_pp
    point = '0 0 175'
    variable = pp 
    block = 'fracture'
  []
[]

[Postprocessors]
  # Periodic pumping postprocessor
  [mass_flux_pp]
    type = FunctionValuePostprocessor
    function = mass_flux_fxn
    execute_on = TIMESTEP_BEGIN
  []  
  [stim_p]
    type = PointValue
    point = '0 0 175'
    variable = pp
    outputs = csv
  []
  [obs4_p]
    type = PointValue
    point = '30 0 175'
    variable = pp
    outputs = csv
  []
  [fluid_in]
    type = PorousFlowFluidMass
    execute_on = TIMESTEP_BEGIN
  []
  [fluid_out]
    type = PorousFlowFluidMass
    execute_on = TIMESTEP_END
  []
  [mass_blnc_pct]
    type = FunctionValuePostprocessor
    function = mass_bal_pct
    execute_on = TIMESTEP_END
  []
[]

[VectorPostprocessors]
  [press_profile]
    type = LineValueSampler
    variable = pp
    start_point = '-80 0 175'
    end_point = '80 0 175'
    num_points = 300
    sort_by = x
    execute_on = 'TIMESTEP_END'
  []
[]

[Preconditioning]
  active = lu
  [lu]
  type = SMP
  full = true
  petsc_options_iname = '-pc_type -pc_factor_mat_solver_package'
  petsc_options_value = 'lu        mumps'
  []
  [basic]
    type = SMP
    full = true
    petsc_options = '-ksp_diagonal_scale -ksp_diagonal_scale_fix'
    petsc_options_iname = '-pc_type -sub_pc_type -sub_pc_factor_shift_type -pc_asm_overlap'
    petsc_options_value = ' asm      lu           NONZERO                   2'
  []
[]

[Executioner]
  type = Transient
  solve_type = NEWTON
  line_search = 'none'
  [TimeSteppers]
    active = constant
    [adaptive]
      type = IterationAdaptiveDT
      dt = 1
      growth_factor = 1.05
      optimal_iterations = 6
    []
    [constant]
      type = ConstantDT
      dt = ${dt}    
    []
  []
  start_time = -${dt}
  end_time = ${endTime}
  l_tol = 1e-08
  l_max_its = 60
  nl_forced_its = 1
  nl_max_its = 40
  nl_abs_tol = 1e-08
[]

[Outputs]
  exodus = false
  csv = true
  execute_on = 'INITIAL TIMESTEP_END'
  file_base = './out_files/${file_name}'
[]