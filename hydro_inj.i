aper_init = 3e-4
dt = 5
endTime = 50
injection_rate = 0.1 # kg/s
p_atm = 101325
file_name = hydro_inj

[Mesh]
  [fmg]
    type = FileMeshGenerator
    file = 'multi_dim_mesh.e'
  []
[]

[GlobalParams]
  PorousFlowDictator = dictator
  multiply_by_density = true
  gravity = '0 0 -9.81'
[]

[UserObjects]
  [dictator]
  type = PorousFlowDictator
  porous_flow_vars = 'pp'
  number_fluid_phases = 1
  number_fluid_components = 1
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
  [hydrostatic]
    family = LAGRANGE
    order = FIRST
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
    type = FunctionIC
    variable = pp
    function = hydrostatic
  []
[]

[Functions]
  [hydrostatic]
    type = ParsedFunction
    expression = 'p_atm + (rho_f*g*(z_max-z))'
    symbol_names = 'p_atm rho_f g z_max'
    symbol_values = '${p_atm} 998.23 9.81 225'
  []
  [rock_mass_bal]
    type = ParsedFunction
    expression = 'abs(in - out)'
    symbol_names = 'in out'
    symbol_values = 'rock_fluid_in rock_fluid_out'
  []
  [frac_mass_bal]
    type = ParsedFunction
    expression = 'abs(in - out)'
    symbol_names = 'in out'
    symbol_values = 'frac_fluid_in frac_fluid_out'
  []
[]

[Kernels]
  # Hydraulic Kernels
  [frac_mass_deriv]
    type = PorousFlowFullySaturatedMassTimeDerivative
    variable = pp
  []
  [darcy_flow]
    type = PorousFlowFullySaturatedDarcyBase
    variable = pp
  []
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
  [temp]
    type = PorousFlowTemperature
  []
  [mass_frac]
    type = PorousFlowMassFraction
  []
  [ppss]
    type = PorousFlow1PhaseFullySaturated
    porepressure = pp
  []
  [eff_press]
    type = PorousFlowEffectiveFluidPressure
  []
  [fluid]
    type = PorousFlowSingleComponentFluid
    fp = water
    phase = 0
  []
  [relp]
    type = PorousFlowRelativePermeabilityConst
    phase = 0
  []
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
  [mass_flux_source]
    type = PorousFlowSquarePulsePointSource
    mass_flux = ${injection_rate}
    point = '0 0 175'
    variable = pp 
    block = 'fracture'
    point_not_found_behavior = WARNING
  []
[]

[Postprocessors]
  [frac_fluid_in]
    type = PorousFlowFluidMass
    execute_on = TIMESTEP_BEGIN
    block = 'fracture'
  []
  [frac_fluid_out]
    type = PorousFlowFluidMass
    execute_on = TIMESTEP_END
    block = 'fracture'
  []
  [frac_mass_blnc]
    type = FunctionValuePostprocessor
    function = frac_mass_bal
    execute_on = TIMESTEP_END
  []
  [rock_fluid_in]
    type = PorousFlowFluidMass
    execute_on = TIMESTEP_BEGIN
    block = 'rock'
  []
  [rock_fluid_out]
    type = PorousFlowFluidMass
    execute_on = TIMESTEP_END
    block = 'rock'
  []
  [rock_mass_blnc]
    type = FunctionValuePostprocessor
    function = rock_mass_bal
    execute_on = TIMESTEP_END
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
  l_tol = 1e-12
  l_max_its = 60
  nl_forced_its = 1
  nl_max_its = 40
  nl_abs_tol = 1e-10
[]

[Outputs]
  exodus = true
  execute_on = 'INITIAL TIMESTEP_END'
  file_base = './out_files/${file_name}'
[]