within ;
model AHUFanMalfunction


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
    annotation (Placement(transformation(extent={{-30,28},{-56,54}})));

  /* Condenser Water SourceMdot and Parameter Set*/
  ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot CondensedWaterIn(
    redeclare package Medium = ThermoCycle.Media.StandardWater,
    Mdot_0=59.75,
    T_0=303.15)
    annotation (Placement(transformation(extent={{-112,52},{-92,72}})));
  /* Condenser Water SinkP and Parameter Set*/
  ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP1(
  redeclare package Medium =
               ThermoCycle.Media.StandardWater)
    annotation (Placement(transformation(extent={{10,52},{30,72}})));
  /* Container and Parameter Set*/
  ThermoCycle.Components.Units.Tanks.Tank_pL Container(
    redeclare package Medium = ThermoCycle.Media.R134a_CP,
    impose_L=true,
    impose_pressure=false,
    Vtot=1000,
    pstart=906000)
    annotation (Placement(transformation(extent={{-100,4},{-80,24}})));
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
        origin={-88,-20})));
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
    annotation (Placement(transformation(extent={{-56,-38},{-30,-64}})));
  /* Chilled Water SourceMdot and Parameter Set*/
  ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot ChilledWaterIn(
    redeclare package Medium = ThermoCycle.Media.StandardWater,
    Mdot_0=47.8,
    T_0=285.15)
    annotation (Placement(transformation(extent={{28,-86},{8,-66}})));
  /* Chilled Water sinkP and Parameter Set*/
  ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP2(redeclare package
      Medium = ThermoCycle.Media.StandardWater)
    annotation (Placement(transformation(extent={{-96,-68},{-116,-48}})));
  /* Compressor and Parameter Set*/
  ThermoCycle.Components.Units.ExpansionAndCompressionMachines.Compressor compressor(
  redeclare package Medium =
               ThermoCycle.Media.R134a_CP,
    V_s=1.787e-3,
    p_su_start=356000,
    p_ex_start=906000,
    T_su_start=279.15)
    annotation (Placement(transformation(extent={{30,10},{6,-14}})));
  /* Electric Drive and Parameter Set*/
  ThermoCycle.Components.Units.ExpansionAndCompressionMachines.ElectricDrive
                                                               electricDrive
    annotation (Placement(transformation(extent={{-8,-12},{-28,8}})));
  /* Sensors*/
  ThermoCycle.Components.FluidFlow.Sensors.SensTp Te_out(redeclare package
      Medium =
        ThermoCycle.Media.R134a_CP)
    annotation (Placement(transformation(extent={{-14,-46},{6,-26}})));
  ThermoCycle.Components.FluidFlow.Sensors.SensTp TEA(redeclare package Medium =
        ThermoCycle.Media.R134a_CP)
    annotation (Placement(transformation(extent={{-82,-46},{-62,-26}})));
  ThermoCycle.Components.FluidFlow.Sensors.SensTp TCA(redeclare package Medium =
        ThermoCycle.Media.R134a_CP)
    annotation (Placement(transformation(extent={{-22,36},{-2,56}})));
  ThermoCycle.Components.FluidFlow.Sensors.SensTp TC_out(redeclare package
      Medium =
        ThermoCycle.Media.R134a_CP)
    annotation (Placement(transformation(extent={{-82,36},{-62,56}})));
  ThermoCycle.Components.FluidFlow.Sensors.SensTp TCI(redeclare package Medium =
        ThermoCycle.Media.StandardWater)
    annotation (Placement(transformation(extent={{-76,62},{-56,82}})));
  ThermoCycle.Components.FluidFlow.Sensors.SensTp TCO(redeclare package Medium =
        ThermoCycle.Media.StandardWater)
    annotation (Placement(transformation(extent={{-30,62},{-10,82}})));
  ThermoCycle.Components.FluidFlow.Sensors.SensTp TEO(redeclare package Medium =
        ThermoCycle.Media.StandardWater)
    annotation (Placement(transformation(extent={{-84,-56},{-64,-76}})));
  ThermoCycle.Components.FluidFlow.Sensors.SensTp TEI(redeclare package Medium =
        ThermoCycle.Media.StandardWater)
    annotation (Placement(transformation(extent={{-32,-56},{-12,-76}})));

  /* Fault Ramps */
  Modelica.Blocks.Sources.Ramp Level_1(
    offset=273.15 + 12,
    duration(displayUnit="d") = 2592000,
    startTime(displayUnit="d") = 2592000,
    height=-1)
    annotation (Placement(transformation(extent={{76,-56},{56,-36}})));
  Modelica.Blocks.Sources.Ramp Level_2(
    offset=273.15 + 12,
    duration(displayUnit="d") = 2592000,
    startTime(displayUnit="d") = 2592000,
    height=-2)
    annotation (Placement(transformation(extent={{112,-56},{92,-36}})));
  Modelica.Blocks.Sources.Ramp Level_3(
    offset=273.15 + 12,
    duration(displayUnit="d") = 2592000,
    startTime(displayUnit="d") = 2592000,
    height=-3)
    annotation (Placement(transformation(extent={{76,-88},{56,-68}})));
  Modelica.Blocks.Sources.Ramp Level_4(
    offset=273.15 + 12,
    duration(displayUnit="d") = 2592000,
    startTime(displayUnit="d") = 2592000,
    height=-4)
    annotation (Placement(transformation(extent={{112,-88},{92,-68}})));

  /* Controller */

  Modelica.Blocks.Sources.Constant DELTAT_SP1(k=273.15 + 7)
    annotation (Placement(transformation(extent={{58,-20},{64,-14}})));
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
    annotation (Placement(transformation(extent={{76,-2},{94,-16}})));
  ThermoCycle.Components.Units.ControlSystems.SH_block sH_block(redeclare
      package Medium =
        ThermoCycle.Media.R134a_CP)
    annotation (Placement(transformation(extent={{38,46},{48,56}})));
  Modelica.Blocks.Sources.Constant DELTAT_SP(k=0.5)
    annotation (Placement(transformation(extent={{44,28},{50,34}})));
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
    annotation (Placement(transformation(extent={{60,42},{78,28}})));

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
      points={{-90,5.2},{-90,-11},{-88,-11}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(electricDrive.shaft, compressor.flange_elc) annotation (Line(
      points={{-9.4,-2},{10,-2}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(Evaporator.outlet_fl1, Te_out.InFlow)
    annotation (Line(points={{-33,-46},{-4,-46},{-4,-45.4}},color={0,0,255}));
  connect(valve.OutFlow, TEA.InFlow) annotation (Line(points={{-88,-29},{-88,-45.4},
          {-72,-45.4}}, color={0,0,255}));
  connect(TEA.InFlow, Evaporator.inlet_fl1) annotation (Line(points={{-72,-45.4},
          {-62,-45.4},{-62,-46},{-53,-46}}, color={0,0,255}));
  connect(Condenser.outlet_fl1, TC_out.InFlow)
    annotation (Line(points={{-53,36},{-72,36},{-72,36.6}}, color={0,0,255}));
  connect(Container.InFlow, TC_out.InFlow) annotation (Line(points={{-90,22.4},{
          -90,36.6},{-72,36.6}},  color={0,0,255}));
  connect(sinkP2.flangeB, TEO.InFlow) annotation (Line(points={{-97.6,-58},{-74,
          -58},{-74,-56.6}}, color={0,0,255}));
  connect(TEO.InFlow, Evaporator.outlet_fl2) annotation (Line(points={{-74,-56.6},
          {-64,-56.6},{-64,-56.8},{-52.8,-56.8}}, color={0,0,255}));
  connect(Evaporator.inlet_fl2, TEI.InFlow) annotation (Line(points={{-33.2,-57},
          {-27.6,-57},{-27.6,-56.6},{-22,-56.6}},
                                              color={0,0,255}));
  connect(TEI.InFlow,ChilledWaterIn. flangeB) annotation (Line(points={{-22,-56.6},
          {-4,-56.6},{-4,-76},{9,-76}},  color={0,0,255}));
  connect(CondensedWaterIn.flangeB, TCI.InFlow)
    annotation (Line(points={{-93,62},{-66,62},{-66,62.6}}, color={0,0,255}));
  connect(TCI.InFlow, Condenser.inlet_fl2) annotation (Line(points={{-66,62.6},{
          -60,62.6},{-60,62},{-56,62},{-56,47},{-52.8,47}}, color={0,0,255}));
  connect(sinkP1.flangeB, TCO.InFlow)
    annotation (Line(points={{11.6,62},{-20,62},{-20,62.6}},
                                                           color={0,0,255}));
  connect(Condenser.outlet_fl2, TCO.InFlow) annotation (Line(points={{-33.2,46.8},
          {-28,46.8},{-28,62.6},{-20,62.6}},
                                          color={0,0,255}));

  connect(compressor.OutFlow, TCA.InFlow) annotation (Line(points={{9.4,2},{8,2},
          {8,34},{8,36.6},{-12,36.6}},     color={0,0,255}));
  connect(TCA.InFlow, Condenser.inlet_fl1) annotation (Line(points={{-12,36.6},{
          -22,36.6},{-22,36},{-33,36}},
                                   color={0,0,255}));

  connect(Te_out.InFlow, compressor.InFlow) annotation (Line(points={{-4,-45.4},
          {14,-45.4},{14,-46},{24.8,-46},{24.8,-9.8}},  color={0,0,255}));
  connect(DELTAT_SP1.y, PID_Compressor.SP) annotation (Line(
      points={{64.3,-17},{69.15,-17},{69.15,-11.8},{76,-11.8}},
      color={0,0,127},
      smooth=Smooth.None,
      pattern=LinePattern.Dot));
  connect(TEO.T, PID_Compressor.PV) annotation (Line(
      points={{-66,-72},{-58,-72},{-58,-92},{-50,-92},{40,-92},{40,-10},{40,-6.2},
          {76,-6.2}},
      color={0,0,127},
      pattern=LinePattern.Dot));
  connect(PID_Compressor.CS, electricDrive.f) annotation (Line(
      points={{94.54,-9},{98,-9},{98,16},{-18.4,16},{-18.4,7.4}},
      color={0,0,127},
      pattern=LinePattern.Dot));
  connect(Te_out.T, sH_block.T_measured) annotation (Line(
      points={{4,-30},{48,-30},{48,20},{30,20},{30,53.5},{37.7,53.5}},
      color={0,0,127},
      smooth=Smooth.None,
      pattern=LinePattern.Dot));
  connect(Te_out.p, sH_block.p_measured) annotation (Line(
      points={{-12,-30},{-24,-30},{-24,-22},{52,-22},{52,24},{34,24},{34,49},{37.9,
          49}},
      color={0,0,127},
      smooth=Smooth.None,
      pattern=LinePattern.Dot));
  connect(DELTAT_SP.y,PID_valve. SP) annotation (Line(
      points={{50.3,31},{55.15,31},{55.15,32.2},{60,32.2}},
      color={0,0,127},
      smooth=Smooth.None,
      pattern=LinePattern.Dot));
  connect(PID_valve.CS, valve.cmd) annotation (Line(points={{78.54,35},{86,35},{
          86,34},{92,34},{92,90},{-120,90},{-120,-20},{-96,-20}},
                                                  color={0,0,127},pattern=LinePattern.Dot));
  connect(sH_block.DeltaT,PID_valve. PV) annotation (Line(points={{48.55,51.25},
          {54,51.25},{54,37.8},{60,37.8}}, color={0,0,127}));
  connect(Level_1.y, ChilledWaterIn.in_T) annotation (Line(points={{55,-46},{46,
          -46},{32,-46},{32,-60},{18.2,-60},{18.2,-70}}, color={0,0,127}));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-140,
            -100},{140,100}})),
    experiment(StopTime=7.776e+006, Interval=600),
    __Dymola_experimentSetupOutput,
    Icon(coordinateSystem(extent={{-100,-100},{100,80}})),
              Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    uses(Modelica(version="3.2.2")));
  annotation (Documentation(info="<HTML>
  <p><big> This model allows the user to simulate the Fault AHU Fan Malfunction in a Chiller system.
    The complete Chiller model is composed by the following components: compressor, evaporator, condenser, 
    one liquid container, a valve, some sensors and controllers.
    <p><big> The fault AHU Fan Malfunction can be simulated by a ramp input connected to the evaporator water inlet Temperature. 
    There are four different levels, the severity is increased by level. The users can change the fault to different level or set the fault as needed.
  
  <p><big> This model is based on ThermoCycle Library (<a href=\"http://www.thermocycle.net/\">http://www.thermocycle.net/</a>) and ExternalMedia Library (<a href=\"https://github.com/modelica/ExternalMedia\">https://github.com/modelica/ExternalMedia</a>).
  It cannot be successully runned without these two Libraries loaded.
  </HTML>"));

end AHUFanMalfunction;
