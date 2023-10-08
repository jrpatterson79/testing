aper_init = 3e-4
E = 50e9
p_atm = 101325
file_name = hydromech_steady

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
  multiply_by_density = false
  use_displaced_mesh = false
  gravity = '0 0 -9.81'
[]

[UserObjects]
  [dictator]
  type = PorousFlowDictator
  porous_flow_vars = 'pp disp_x disp_y disp_z'
  number_fluid_phases = 1
  number_fluid_components = 1
  []
[]

[Variables]
  [pp]
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
  [stress_xx]
    family = MONOMIAL
    order = CONSTANT
    block = 'rock'
  []
  [stress_xy]
    family = MONOMIAL
    order = CONSTANT
    block = 'rock'
  []
  [stress_xz]
    family = MONOMIAL
    order = CONSTANT
    block = 'rock'
  []
  [stress_yy]
    family = MONOMIAL
    order = CONSTANT
    block = 'rock'
  []
  [stress_yz]
    family = MONOMIAL
    order = CONSTANT
    block = 'rock'
  []
  [stress_zz]
    family = MONOMIAL
    order = CONSTANT
    block = 'rock'
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
  #Mechanical Boundaries
  [roller_x]
    type = NeumannBC
    variable = disp_x
    boundary = 'x_min x_max'
    value = 0
  []
  [roller_y]
    type = NeumannBC
    variable = disp_y
    boundary = 'y_min y_max'
    value = 0
  []
  [z_load]
    type = FunctionNeumannBC
    variable = disp_z
    boundary = 'z_max'
    function = top_load
  []
  [zero_disp]
    type = DirichletBC
    variable = disp_z
    boundary = 'z_min'
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
  # Hydrostatic pressure gradient
  [hydrostatic]
    type = ParsedFunction
    expression = 'p_atm + (rho_f*g*(z_max-z))'
    symbol_names = 'p_atm rho_f g z_max'
    symbol_values = '${p_atm} 998.23 9.81 225'
  []
  # Initial stress gradient
  [stress_init]
    type = ParsedFunction
    expression = '- ((rho_rock*g*(z_max-z)) - (p_atm+(rho_f*g*1*(z_max-z))))'
    symbol_names = 'rho_rock rho_f g p_atm z_max'
    symbol_values = '2750 998.23 9.81 ${p_atm} 225'
  []
  [top_load]
    type = ParsedFunction
    expression = '- (p_atm + (rho_seds * g * h_seds))'
    symbol_names = 'p_atm rho_seds h_seds g'
    symbol_values = '${p_atm} 2500 15 9.81'
  []
  [mass_bal_pct]
    type = ParsedFunction
    expression = '((in - out)/in) * 100'
    symbol_names = 'in out'
    symbol_values = 'fluid_in fluid_out'
  []
[]

[Kernels]
  # Fracture Hydraulic Kernels
  [darcy_flow]
    type = PorousFlowFullySaturatedDarcyBase
    variable = pp
  []
  # Mechanical Kernels
  [grad_stress_x]
    type = StressDivergenceTensors
    displacements = 'disp_x disp_y disp_z'
    variable = disp_x
    component = 0
    block = 'rock'
  []
  [grad_stress_y]
    type = StressDivergenceTensors
    displacements = 'disp_x disp_y disp_z'
    variable = disp_y
    component = 1
    block = 'rock'
  []
  [grad_stress_z]
    type = StressDivergenceTensors
    displacements = 'disp_x disp_y disp_z'
    variable = disp_z
    component = 2
    block = 'rock'
  []
  [poro_x]
    type = PorousFlowEffectiveStressCoupling
    variable = disp_x
    component = 0
    block = 'rock'
  []
  [poro_y]
    type = PorousFlowEffectiveStressCoupling
    variable = disp_y
    component = 1
    block = 'rock'
  []
  [poro_z]
    type = PorousFlowEffectiveStressCoupling
    variable = disp_z
    component = 2
    block = 'rock'
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
  # Hydraulic materials
  [temp]
    type = PorousFlowTemperature
  []
  [mass_frac]
    type = PorousFlowMassFraction
  []
  [fluid]
    type = PorousFlowSingleComponentFluid
    fp = water
    phase = 0
  []
  [ppss]
    type = PorousFlow1PhaseFullySaturated
    porepressure = pp
  []
  [eff_press]
    type = PorousFlowEffectiveFluidPressure
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
  [relp]
    type = PorousFlowRelativePermeabilityConst
    phase = 0
  []
  # Mechanical Materials
  [elasticity_tensor]
    type = ComputeIsotropicElasticityTensor
    youngs_modulus = ${E}
    poissons_ratio = 0.25
    block = 'rock'
  []
  [initial_strain]
    type = ComputeEigenstrainFromInitialStress
    initial_stress = 'stress_init 0 0   0 stress_init 0   0 0 stress_init'
    eigenstrain_name = initial_stress
    block = 'rock'
  []
  [strain]
    type = ComputeSmallStrain
    displacements = 'disp_x disp_y disp_z'
    eigenstrain_names = 'initial_stress'
    block = 'rock'
  []
  [vol_strain]
    type = PorousFlowVolumetricStrain
    displacements = 'disp_x disp_y disp_z'
    block = 'rock'
  []
  [stress]
    type = ComputeLinearElasticStress
    block = 'rock'
  []
  [biot_mod]
    type = PorousFlowConstantBiotModulus
    biot_coefficient = 1
  []
  [density]
    type = GenericConstantMaterial
    prop_names = density
    prop_values = 2750
    block = 'rock'
  []
[]

[Postprocessors]
  [p0]
    type = PointValue
    point = '0 30 175'
    variable = pp
    outputs = csv
  []
  [z0]
    type = PointValue
    point = '0 30 175'
    variable = disp_z
    outputs = csv
  []
  [fluid_in]
    type = PorousFlowFluidMass
    execute_on = TIMESTEP_BEGIN
    block = 'rock'
  []
  [fluid_out]
    type = PorousFlowFluidMass
    execute_on = TIMESTEP_END
    block = 'rock'
  []
  [mass_blnc_pct]
    type = FunctionValuePostprocessor
    function = mass_bal_pct
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
  type = Steady
  solve_type = NEWTON
  line_search = 'none'
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