endTime = 1e3

[Mesh]
  uniform_refine = 0
  [rock_mesh]
    type = GeneratedMeshGenerator
    dim = 3
    xmin = -375
    xmax = 375
    ymin = -375
    ymax = 375
    zmin = -150
    zmax = 150
    nx = 25
    ny = 25
    nz = 10
  []
[]

[Variables]
  [pp]
    initial_condition = 0
  []
  [disp_x]
    scaling = 1e-10
  []
  [disp_y]
    scaling = 1e-10
  []
  [disp_z]
    scaling = 1e-10
  []
[]

[GlobalParams]
  PorousFlowDictator = dictator
[]

[AuxVariables]
  [fracture_normal_x]
    family = MONOMIAL
    order = CONSTANT
    # initial_condition = 0
  []
  [fracture_normal_y]
    family = MONOMIAL
    order = CONSTANT
    # initial_condition = 1
  []
  [fracture_normal_z]
    family = MONOMIAL
    order = CONSTANT
    # initial_condition = 0
  []
  [element_normal_length]
    family = MONOMIAL
    order = CONSTANT
  []
  [pp_from_frac]
    family = MONOMIAL
    order = CONSTANT
  []
  [stress_xx]
    family = MONOMIAL
    order = CONSTANT
  []
  [stress_xy]
    family = MONOMIAL
    order = CONSTANT
  []
  [stress_xz]
    family = MONOMIAL
    order = CONSTANT
  []
  [stress_yy]
    family = MONOMIAL
    order = CONSTANT
  []
  [stress_yz]
    family = MONOMIAL
    order = CONSTANT
  []
  [stress_zz]
    family = MONOMIAL
    order = CONSTANT
  []
[]

[AuxKernels]
  [stress_xx]
    type = RankTwoAux
    rank_two_tensor = stress
    variable = stress_xx
    index_i = 0
    index_j = 0
  []
  [stress_xy]
    type = RankTwoAux
    rank_two_tensor = stress
    variable = stress_xy
    index_i = 0
    index_j = 1
  []
  [stress_xz]
    type = RankTwoAux
    rank_two_tensor = stress
    variable = stress_xz
    index_i = 0
    index_j = 2
  []
  [stress_yy]
    type = RankTwoAux
    rank_two_tensor = stress
    variable = stress_yy
    index_i = 1
    index_j = 1
  []
  [stress_yz]
    type = RankTwoAux
    rank_two_tensor = stress
    variable = stress_yz
    index_i = 1
    index_j = 2
  []
  [stress_zz]
    type = RankTwoAux
    rank_two_tensor = stress
    variable = stress_zz
    index_i = 2
    index_j = 2
  []
[]

[BCs]
  [roller_x]
    type = DirichletBC
    variable = disp_x
    boundary = 'left right'
    value = 0
  []
  [roller_y]
    type = DirichletBC
    variable = disp_y
    boundary = 'front back'
    value = 0
  []
  [const_press]
    type = DirichletBC
    variable = pp
    boundary = 'left right front back'
    value = 0
  []
  [no_flow]
    type = NeumannBC
    variable = pp
    boundary = 'top bottom' 
    value = 0
  []
[]

[FluidProperties]
  [water]
    type = SimpleFluidProperties
    viscosity = 1e-3
  []
[]

[Materials]
  [porosity]
    type = PorousFlowPorosityConst
    porosity = 0.02
  []  
  [permeability]
    type = PorousFlowPermeabilityConst
    permeability = '1e-18 0 0   0 1e-18 0  0 0 1e-18'
  []
  [elasticity_tensor]
    type = ComputeIsotropicElasticityTensor
    youngs_modulus = 3.795e10
    poissons_ratio = 0.10
  []
  [strain]
    type = ComputeSmallStrain
    displacements = 'disp_x disp_y disp_z'
  []
  [stress]
    type = ComputeLinearElasticStress
  []
  [biot_mod]
    type = PorousFlowConstantBiotModulus
    fluid_bulk_modulus = 2e9
    biot_coefficient = 1
  []
[]

[PorousFlowFullySaturated]
  coupling_type = HydroMechanical
  dictator_name = dictator
  multiply_by_density = true
  use_displaced_mesh = false
  displacements = 'disp_x disp_y disp_z'
  porepressure = pp
  fp = water
  gravity = '0 0 0'
  pressure_unit = Pa
  stabilization = Full
[]

[DiracKernels]
  [pp_from_frac]
    type = ReporterPointSource
    variable = pp
    value_name = frac_p/transferred_frac_p
    x_coord_name = frac_p/x
    y_coord_name = frac_p/y
    z_coord_name = frac_p/z
  []
[]

[Reporters]
  [frac_p]
    type = ConstantReporter
    real_vector_names = 'transferred_frac_p x y z'
    real_vector_values = '0; 0; 0; 0'
    outputs = none
  []
[]

[Preconditioning]
  [superlu]
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
    type = IterationAdaptiveDT
    dt = 1
    growth_factor = 1.05
    optimal_iterations = 6
  []
  end_time = ${endTime}
  fixed_point_max_its = 10
  l_max_its = 60
  nl_forced_its = 1
  nl_max_its = 40
  nl_abs_tol = 1e-12
[]

[Outputs]
  print_linear_residuals = false
  file_base = rock_app
  exodus = true
  csv = true
[]

[MultiApps]
  [frac_app]
    type = TransientMultiApp
    input_files = frac_app.i
    execute_on = TIMESTEP_BEGIN
    sub_cycling = false
  []
[]

[Transfers]
  [normal_x_from_fracture]
    type = MultiAppNearestNodeTransfer
    from_multi_app = frac_app
    source_variable = normal_dirn_x
    variable = fracture_normal_x
  []
  [normal_y_from_fracture]
    type = MultiAppNearestNodeTransfer
    from_multi_app = frac_app
    source_variable = normal_dirn_y
    variable = fracture_normal_y
  []
  [normal_z_from_fracture]
    type = MultiAppNearestNodeTransfer
    from_multi_app = frac_app
    source_variable = normal_dirn_z
    variable = fracture_normal_z
  []
  [pp_from_frac]
    type = MultiAppReporterTransfer
    from_multi_app = frac_app
    from_reporters = 'frac_p/pp frac_p/x frac_p/y frac_p/z'
    to_reporters = 'frac_p/transferred_frac_p frac_p/x frac_p/y frac_p/z'
  []
[]