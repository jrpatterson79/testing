endTime = 1e3
injection_rate = 1

[Mesh]
  uniform_refine = 0
  [frac_mesh]
    type = FileMeshGenerator
    file = 'sing_frac.msh'
  []
[]

[GlobalParams]
  PorousFlowDictator = dictator
[]

[Variables]
  [pp]
    initial_condition = 0
  []
[]

[AuxVariables]
  [normal_dirn_x]
    family = MONOMIAL
    order = CONSTANT
  []
  [normal_dirn_y]
    family = MONOMIAL
    order = CONSTANT
  []
  [normal_dirn_z]
    family = MONOMIAL
    order = CONSTANT
  []
  [enclosing_element_normal_length]
    family = MONOMIAL
    order = CONSTANT
  []
[]

[AuxKernels]
    [normal_dirn_x]
      type = PorousFlowElementNormal
      variable = normal_dirn_x
      component = x
    []
    [normal_dirn_y]
      type = PorousFlowElementNormal
      variable = normal_dirn_y
      component = y
    []
    [normal_dirn_z]
      type = PorousFlowElementNormal
      variable = normal_dirn_z
      component = z
    []
[]

[PorousFlowFullySaturated]
  dictator_name = dictator
  multiply_by_density = true
  porepressure = pp
  fp = water
  gravity = '0 0 0'
  pressure_unit = Pa
  stabilization = Full
[]

[FluidProperties]
  [water]
    type = SimpleFluidProperties
    viscosity = 1e-3
  []
[]

[DiracKernels]
  [mass_flux_source]
    type = PorousFlowSquarePulsePointSource
    mass_flux = ${injection_rate}
    point = '0 0 0'
    variable = pp
  []
[]

[Materials]
  [porosity]
    type = PorousFlowPorosityConst
    porosity = 0.1
  []
  [permeability]
    type = PorousFlowPermeabilityConst
    permeability = '1e-12 0 0   0 1e-12 0   0 0 1e-12'
  []
  [biot_mod]
    type = PorousFlowConstantBiotModulus
    fluid_bulk_modulus = 2e9
    biot_coefficient = 1
  []
[]

[Preconditioning]
  [lu]
  type = SMP
  full = true
  petsc_options_iname = '-pc_type -pc_factor_mat_solver_package'
  petsc_options_value = 'lu        mumps'
  []
[]

[Executioner]
  type = Transient
  solve_type = NEWTON
  [TimeStepper]
    type = IterationAdaptiveDT #ConstantDT
    dt = 1
    growth_factor = 1.05
    optimal_iterations = 6
  []
  end_time = ${endTime}
  l_max_its = 60
  nl_forced_its = 1
  nl_max_its = 40
  nl_abs_tol = 1e-12
[]

[VectorPostprocessors]
  [frac_p]
    type = NodalValueSampler
    outputs = none
    sort_by = id
    variable = pp
  []
[]

[Outputs]
  print_linear_residuals = false
  file_base = frac_app
  execute_on = 'timestep_begin'
  exodus = true
  csv = true
[]