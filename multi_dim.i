[Mesh]
  [fmg]
    type = FileMeshGenerator
    file = 'multi_dim_mesh.exo'
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
  gravity = '0 0 0'
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
    initial_condition = 20e6
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
  [velocity_x]
    type = PorousFlowDarcyVelocityComponentLowerDimensional
    variable = velocity_x
    component = x
    aperture = 3E-4
  []
  [velocity_y]
    type = PorousFlowDarcyVelocityComponentLowerDimensional
    variable = velocity_y
    component = y
    aperture = 3E-4
  []
  [velocity_z]
    type = PorousFlowDarcyVelocityComponentLowerDimensional
    variable = velocity_z
    component = z
    aperture = 3E-4
  []
  [stress_xx]
    type = RankTwoAux
    rank_two_tensor = stress
    variable = stress_xx
    index_i = 0
    index_j = 0
    block = 2
  []
  [stress_xy]
    type = RankTwoAux
    rank_two_tensor = stress
    variable = stress_xy
    index_i = 0
    index_j = 1
    block = 2
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
    block = 2
  []
  [stress_yz]
    type = RankTwoAux
    rank_two_tensor = stress
    variable = stress_yz
    index_i = 1
    index_j = 2
    block = 2
  []
  [stress_zz]
    type = RankTwoAux
    rank_two_tensor = stress
    variable = stress_zz
    index_i = 2
    index_j = 2
    block = 2
  []
[]

[DiracKernels]
  [mass_flux_source]
    type = PorousFlowSquarePulsePointSource
    mass_flux = 10
    point = '0 0 0'
    block = 1
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
  [fluid]
    type = PorousFlowSingleComponentFluid
    fp = water
    phase = 0
  []
  [ppss]
    type = PorousFlow1PhaseFullySaturated
    porepressure = pp
  []
  [frac_porosity]
    type = PorousFlowPorosityConst
    porosity = 3e-4
    block = 1
  []
  [rock_porosity]
    type = PorousFlowPorosityConst
    porosity = 0.02
    block = 2
  []
  [frac_perm]
    type = PorousFlowPermeabilityConst
    permeability = '1e-12 0 0   0 1e-12 0   0 0 1e-12'
    block = 1
  []
  [rock_perm]
    type = PorousFlowPermeabilityConst
    permeability = '1e-18 0 0   0 1e-18 0   0 0 1e-18'
    block = 2
  []
  [relp]
    type = PorousFlowRelativePermeabilityConst
    phase = 0
  []
  [elasticity_tensor]
    type = ComputeIsotropicElasticityTensor
    youngs_modulus = 3.795e10
    poissons_ratio = 0.10
    block = 2
  []
  [strain]
    type = ComputeSmallStrain
    displacements = 'disp_x disp_y disp_z'
    block = 2
  []
  [stress]
    type = ComputeLinearElasticStress
    block = 2
  []
  [biot_mod]
    type = GenericConstantMaterial
    prop_names = biot_coefficient
    prop_values = 0.5
    block = 2
  []
[]

[Kernels]
  [time_deriv]
    type = PorousFlowMassTimeDerivative
    fluid_component = 0
    variable = pp
  []
  [darcy_flow]
    type = PorousFlowAdvectiveFlux
    fluid_component = 0
    variable = pp
  []
  [grad_stress_x]
    type = StressDivergenceTensors
    displacements = 'disp_x disp_y disp_z'
    variable = disp_x
    component = 0
    block = 2
  []
  [grad_stress_y]
    type = StressDivergenceTensors
    displacements = 'disp_x disp_y disp_z'
    variable = disp_y
    component = 1
    block = 2
  []
  [grad_stress_z]
    type = StressDivergenceTensors
    displacements = 'disp_x disp_y disp_z'
    variable = disp_z
    component = 2
    block = 2
  []
  [poro_x]
    type = PoroMechanicsCoupling
    variable = disp_x
    porepressure = pp
    component = 0
    block = 2
  []
  [poro_y]
    type = PoroMechanicsCoupling
    variable = disp_y
    porepressure = pp
    component = 1
    block = 2
  []
  [poro_z]
    type = PoroMechanicsCoupling
    variable = disp_z
    porepressure = pp
    component = 2
    block = 2
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
  end_time = 1e1
  # fixed_point_max_its = 10
  l_max_its = 60
  nl_forced_its = 1
  nl_max_its = 40
  nl_abs_tol = 1e-12
[]

[Outputs]
  exodus = true
  file_base = multi_dim_test
[]