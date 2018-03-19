within ;
model ReducedCondFlow

  //Variables
  Real kw "Compressor Power";
  Real PRE "Evaporator Pressure";
  Real PRC "Condenser Pressure";
  Modelica.SIunits.Temperature TRC_subT "Condenser Outlet Temperature";
  Modelica.SIunits.Temperature Tsh_sucT "Evaporator Outlet Temperature";
  Modelica.SIunits.Temperature TEAT "Evaporator Approach Temperature";
  Modelica.SIunits.Temperature TCAT "Condenser Approach Temperature";
  Real TEITEO "Evaporator Water Temperature Difference";
  Real TCOTCI "Condenser Water Temperature Difference";
  Real kW_ton "Chiller Efficiency";
  Real COP "Coefficient of Performance";
  Real Q_c "Condenser Capacity";
  Real Q_e "Evaporator Capacity";

  /* Condenser and Parameter Set */
  ThermoCycle.Components.Units.HeatExchangers.Hx1DInc Condenser(
    redeclare package Medium1 = ThermoCycle.Media.R134a_CP,
    redeclare package Medium2 = ThermoCycle.Media.StandardWater,
    N=10,
    redeclare model Medium1HeatTransferModel =
        ThermoCycle.Components.HeatFlow.HeatTransfer.Constant,
    M_wall=10,
    Unom_l=4000,
    Unom_tp=4000,
    Unom_v=4000,
    counterCurrent=true,
    Mdotnom_sf=59.75,
    Mdotnom_wf=6.7,
    Unom_sf=1160,
    V_sf=11,
    V_wf=11,
    A_sf=430,
    A_wf=430,
    Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind,
    Mdotconst_wf=false,
    pstart_sf=101325,
    pstart_wf=915000,
    Tstart_inlet_wf=309.15,
    Tstart_outlet_wf=309.15,
    Tstart_inlet_sf=303.15,
    Tstart_outlet_sf=303.15)
    annotation (Placement(transformation(extent={{0,26},{-26,52}})));

  /* Condenser Water SourceMdot and Parameter Set*/
  ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot CondensedWaterIn(
    redeclare package Medium = ThermoCycle.Media.StandardWater,
    Mdot_0=59.75,
    T_0=303.15)
    annotation (Placement(transformation(extent={{-82,50},{-62,70}})));
  /* Condenser Water SinkP and Parameter Set*/
  ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP1(
  redeclare package Medium =
               ThermoCycle.Media.StandardWater)
    annotation (Placement(transformation(extent={{40,50},{60,70}})));
  /* Container and Parameter Set*/
  ThermoCycle.Components.Units.Tanks.Tank_pL Container(
    redeclare package Medium = ThermoCycle.Media.R134a_CP,
    impose_L=true,
    impose_pressure=false,
    Vtot=1000,
    pstart=906000)
    annotation (Placement(transformation(extent={{-70,2},{-50,22}})));
  /* Expansion Valve and Parameter Set*/
  ThermoCycle.Components.Units.PdropAndValves.Valve             valve(
    redeclare package Medium = ThermoCycle.Media.R134a_CP,
    Mdot_nom=0.044,
    UseNom=false,
    use_rho_nom=true,
    Xopen=0.606,
    Afull=3.0e-4,
    p_nom=1650000,
    T_nom=308.15,
    DELTAp_nom=1200000)
    annotation (Placement(transformation(extent={{10,-10},{-10,10}},
        rotation=90,
        origin={-58,-22})));
  /* Evaporator and Parameter Set*/
  ThermoCycle.Components.Units.HeatExchangers.Hx1DInc Evaporator(
    redeclare package Medium1 = ThermoCycle.Media.R134a_CP,
    redeclare package Medium2 = ThermoCycle.Media.StandardWater,
    redeclare model Medium1HeatTransferModel =
        ThermoCycle.Components.HeatFlow.HeatTransfer.Constant,
    M_wall=10,
    counterCurrent=true,
    Unom_l=4000,
    Unom_tp=4000,
    Unom_v=4000,
    Unom_sf=1160,
    Mdotnom_sf=47.8,
    Mdotnom_wf=6.7,
    N=10,
    steadystate_T_sf=false,
    V_sf=10,
    V_wf=10,
    steadystate_T_wall=false,
    Discretization=ThermoCycle.Functions.Enumerations.Discretizations.upwind,
    Mdotconst_wf=false,
    A_sf=350,
    A_wf=350,
    pstart_wf=356000,
    Tstart_inlet_wf=278.15,
    Tstart_outlet_wf=278.15,
    Tstart_inlet_sf=285.15,
    Tstart_outlet_sf=285.15)
    annotation (Placement(transformation(extent={{-26,-40},{0,-66}})));
  /* Chilled Water SourceMdot and Parameter Set*/
  ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot ChilledWaterIn(
    redeclare package Medium = ThermoCycle.Media.StandardWater,
    Mdot_0=47.8,
    T_0=285.15)
    annotation (Placement(transformation(extent={{58,-88},{38,-68}})));
  /* Chilled Water sinkP and Parameter Set*/
  ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP2(redeclare package
      Medium = ThermoCycle.Media.StandardWater)
    annotation (Placement(transformation(extent={{-66,-70},{-86,-50}})));
  /* Compressor and Parameter Set*/
  ThermoCycle.Components.Units.ExpansionAndCompressionMachines.Compressor compressor(
  redeclare package Medium =
               ThermoCycle.Media.R134a_CP,
    V_s=1.787e-3,
    p_su_start=356000,
    p_ex_start=906000,
    T_su_start=279.15)
    annotation (Placement(transformation(extent={{60,8},{36,-16}})));
  /* Electric Drive and Parameter Set*/
  ThermoCycle.Components.Units.ExpansionAndCompressionMachines.ElectricDrive
                                                               electricDrive
    annotation (Placement(transformation(extent={{22,-14},{2,6}})));
  /* Sensors*/
  ThermoCycle.Components.FluidFlow.Sensors.SensTp Te_out(redeclare package
      Medium =
        ThermoCycle.Media.R134a_CP)
    annotation (Placement(transformation(extent={{16,-48},{36,-28}})));
  ThermoCycle.Components.FluidFlow.Sensors.SensTp TEA(redeclare package Medium =
        ThermoCycle.Media.R134a_CP)
    annotation (Placement(transformation(extent={{-52,-48},{-32,-28}})));
  ThermoCycle.Components.FluidFlow.Sensors.SensTp TCA(redeclare package Medium =
        ThermoCycle.Media.R134a_CP)
    annotation (Placement(transformation(extent={{8,34},{28,54}})));
  ThermoCycle.Components.FluidFlow.Sensors.SensTp TC_out(redeclare package
      Medium =
        ThermoCycle.Media.R134a_CP)
    annotation (Placement(transformation(extent={{-52,34},{-32,54}})));
  ThermoCycle.Components.FluidFlow.Sensors.SensTp TCI(redeclare package Medium =
        ThermoCycle.Media.StandardWater)
    annotation (Placement(transformation(extent={{-46,60},{-26,80}})));
  ThermoCycle.Components.FluidFlow.Sensors.SensTp TCO(redeclare package Medium =
        ThermoCycle.Media.StandardWater)
    annotation (Placement(transformation(extent={{0,60},{20,80}})));
  ThermoCycle.Components.FluidFlow.Sensors.SensTp TEO(redeclare package Medium =
        ThermoCycle.Media.StandardWater)
    annotation (Placement(transformation(extent={{-54,-58},{-34,-78}})));
  ThermoCycle.Components.FluidFlow.Sensors.SensTp TEI(redeclare package Medium =
        ThermoCycle.Media.StandardWater)
    annotation (Placement(transformation(extent={{-2,-58},{18,-78}})));
  /* Fault Ramp */
  Modelica.Blocks.Sources.Ramp Level_4(
    duration(displayUnit="d") = 2592000,
    startTime(displayUnit="d") = 2592000,
    offset=59.75,
    height=-23.9)
    annotation (Placement(transformation(extent={{-130,-24},{-110,-4}})));
  Modelica.Blocks.Sources.Ramp Level_1(
    duration(displayUnit="d") = 2592000,
    startTime(displayUnit="d") = 2592000,
    offset=59.75,
    height=-5.975)
    annotation (Placement(transformation(extent={{-130,68},{-110,88}})));
  Modelica.Blocks.Sources.Ramp Level_2(
    duration(displayUnit="d") = 2592000,
    startTime(displayUnit="d") = 2592000,
    offset=59.75,
    height=-11.95)
    annotation (Placement(transformation(extent={{-130,38},{-110,58}})));
  Modelica.Blocks.Sources.Ramp Level_3(
    duration(displayUnit="d") = 2592000,
    startTime(displayUnit="d") = 2592000,
    offset=59.75,
    height=-17.925)
    annotation (Placement(transformation(extent={{-130,8},{-110,28}})));


  /* Controller */

  Modelica.Blocks.Sources.Constant DELTAT_SP1(k=273.15 + 7)
    annotation (Placement(transformation(extent={{88,-22},{94,-16}})));
  ThermoCycle.Components.Units.ControlSystems.PID PID_Compressor(
    PVmin=1,
    Td=0,
    steadyStateInit=false,
    Ti=0,
    Kp=-400,
    PVmax=15,
    PVstart=273.15 + 7,
    CSmin=166.7,
    CSmax=500,
    CSstart=424.824)
    annotation (Placement(transformation(extent={{106,-4},{124,-18}})));
  ThermoCycle.Components.Units.ControlSystems.SH_block sH_block(redeclare
      package Medium =
        ThermoCycle.Media.R134a_CP)
    annotation (Placement(transformation(extent={{68,44},{78,54}})));
  Modelica.Blocks.Sources.Constant DELTAT_SP(k=0.5)
    annotation (Placement(transformation(extent={{74,26},{80,32}})));
  ThermoCycle.Components.Units.ControlSystems.PID             PID_valve(
    CSmax=1,
    CSmin=0,
    PVmin=1,
    PVmax=10,
    Td=0,
    steadyStateInit=false,
    Ti=0,
    PVstart=3,
    Kp=-400,
    CSstart=0.606)
    annotation (Placement(transformation(extent={{90,40},{108,26}})));

equation
    kw = compressor.W_dot;
    PRE = Evaporator.Summary.p_wf;
    PRC = Condenser.Summary.p_wf;
    TRC_subT =TC_out.T;
    Tsh_sucT =Te_out.T;
    TEAT = TEA.T;
    TCAT = TCA.T;
    TEITEO = TEI.T - TEO.T;
    TCOTCI = TCO.T - TCI.T;
    kW_ton = compressor.W_dot/Evaporator.Q_wf_;
    COP = Evaporator.Q_wf_/compressor.W_dot;
    Q_c = - Condenser.Q_wf_;
    Q_e = Evaporator.Q_wf_;

  connect(Container.OutFlow, valve.InFlow) annotation (Line(
      points={{-60,3.2},{-60,-4},{-60,-13},{-58,-13}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(electricDrive.shaft, compressor.flange_elc) annotation (Line(
      points={{20.6,-4},{40,-4}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(Evaporator.outlet_fl1, Te_out.InFlow)
    annotation (Line(points={{-3,-48},{26,-48},{26,-47.4}}, color={0,0,255}));
  connect(valve.OutFlow, TEA.InFlow) annotation (Line(points={{-58,-31},{-58,-47.4},
          {-42,-47.4}}, color={0,0,255}));
  connect(TEA.InFlow, Evaporator.inlet_fl1) annotation (Line(points={{-42,-47.4},
          {-32,-47.4},{-32,-48},{-23,-48}}, color={0,0,255}));
  connect(Condenser.outlet_fl1, TC_out.InFlow)
    annotation (Line(points={{-23,34},{-42,34},{-42,34.6}}, color={0,0,255}));
  connect(Container.InFlow, TC_out.InFlow) annotation (Line(points={{-60,20.4},
          {-60,34.6},{-42,34.6}}, color={0,0,255}));
  connect(sinkP2.flangeB, TEO.InFlow) annotation (Line(points={{-67.6,-60},{-44,
          -60},{-44,-58.6}}, color={0,0,255}));
  connect(TEO.InFlow, Evaporator.outlet_fl2) annotation (Line(points={{-44,
          -58.6},{-34,-58.6},{-34,-58.8},{-22.8,-58.8}},
                                                  color={0,0,255}));
  connect(Evaporator.inlet_fl2, TEI.InFlow) annotation (Line(points={{-3.2,-59},
          {2.4,-59},{2.4,-58.6},{8,-58.6}},   color={0,0,255}));
  connect(TEI.InFlow,ChilledWaterIn. flangeB) annotation (Line(points={{8,-58.6},
          {26,-58.6},{26,-78},{39,-78}}, color={0,0,255}));
  connect(CondensedWaterIn.flangeB, TCI.InFlow)
    annotation (Line(points={{-63,60},{-36,60},{-36,60.6}}, color={0,0,255}));
  connect(TCI.InFlow, Condenser.inlet_fl2) annotation (Line(points={{-36,60.6},
          {-30,60.6},{-30,60},{-26,60},{-26,45},{-22.8,45}},color={0,0,255}));
  connect(sinkP1.flangeB, TCO.InFlow)
    annotation (Line(points={{41.6,60},{10,60},{10,60.6}}, color={0,0,255}));
  connect(Condenser.outlet_fl2, TCO.InFlow) annotation (Line(points={{-3.2,44.8},
          {2,44.8},{2,60.6},{10,60.6}},   color={0,0,255}));

  connect(compressor.OutFlow, TCA.InFlow) annotation (Line(points={{39.4,0},{38,
          0},{38,32},{38,34.6},{18,34.6}}, color={0,0,255}));
  connect(TCA.InFlow, Condenser.inlet_fl1) annotation (Line(points={{18,34.6},{
          8,34.6},{8,34},{-3,34}}, color={0,0,255}));

  connect(Te_out.InFlow, compressor.InFlow) annotation (Line(points={{26,-47.4},
          {44,-47.4},{44,-48},{54.8,-48},{54.8,-11.8}}, color={0,0,255}));
  connect(DELTAT_SP1.y, PID_Compressor.SP) annotation (Line(
      points={{94.3,-19},{99.15,-19},{99.15,-13.8},{106,-13.8}},
      color={0,0,127},
      smooth=Smooth.None,
      pattern=LinePattern.Dot));
  connect(TEO.T, PID_Compressor.PV) annotation (Line(
      points={{-36,-74},{-28,-74},{-28,-94},{-20,-94},{70,-94},{70,-12},{70,
          -8.2},{106,-8.2}},
      color={0,0,127},
      pattern=LinePattern.Dot));
  connect(PID_Compressor.CS, electricDrive.f) annotation (Line(
      points={{124.54,-11},{128,-11},{128,14},{11.6,14},{11.6,5.4}},
      color={0,0,127},
      pattern=LinePattern.Dot));
  connect(Te_out.T, sH_block.T_measured) annotation (Line(
      points={{34,-32},{78,-32},{78,18},{60,18},{60,51.5},{67.7,51.5}},
      color={0,0,127},
      smooth=Smooth.None,
      pattern=LinePattern.Dot));
  connect(Te_out.p, sH_block.p_measured) annotation (Line(
      points={{18,-32},{6,-32},{6,-24},{82,-24},{82,22},{64,22},{64,47},{67.9,
          47}},
      color={0,0,127},
      smooth=Smooth.None,
      pattern=LinePattern.Dot));
  connect(DELTAT_SP.y,PID_valve. SP) annotation (Line(
      points={{80.3,29},{85.15,29},{85.15,30.2},{90,30.2}},
      color={0,0,127},
      smooth=Smooth.None,
      pattern=LinePattern.Dot));
  connect(PID_valve.CS, valve.cmd) annotation (Line(points={{108.54,33},{116,33},
          {116,32},{122,32},{122,88},{-90,88},{-90,-22},{-66,-22}},
                                                  color={0,0,127},pattern=LinePattern.Dot));
  connect(sH_block.DeltaT,PID_valve. PV) annotation (Line(points={{78.55,49.25},
          {84,49.25},{84,35.8},{90,35.8}}, color={0,0,127}));
  connect(Level_1.y, CondensedWaterIn.in_Mdot)
    annotation (Line(points={{-109,78},{-78,78},{-78,66}}, color={0,0,127}));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-140,
            -100},{140,100}})),
    experiment(StopTime=7.776e+006, Interval=600),
    __Dymola_experimentSetupOutput,
    Icon(coordinateSystem(extent={{-100,-100},{100,80}})),
              Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    uses(Modelica(version="3.2.2")));
  annotation (Documentation(info="<HTML>
  <p><big> This model allows the user to simulate the Fault Reduced Condenser Water Flow in a Chiller system.
    The complete Chiller model is composed by the following components: compressor, evaporator, condenser, 
    one liquid container, a valve, some sensors and controllers.
    <p><big> The fault Reduced Condenser Water Flow can be simulated by a ramp input connected to the condenser water inlet Mdot. 
    There are four different levels, the severity is increased by level. The users can change the fault to different level or set the fault as needed.
  
  <p><big> This model is based on ThermoCycle Library (<a href=\"http://www.thermocycle.net/\">http://www.thermocycle.net/</a>) and ExternalMedia Library (<a href=\"https://github.com/modelica/ExternalMedia\">https://github.com/modelica/ExternalMedia</a>).
  It cannot be successully runned without these two Libraries loaded.
  </HTML>"));
end ReducedCondFlow;
