within ;
model ReducedEvapFlow



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
    annotation (Placement(transformation(extent={{-18,28},{-44,54}})));

  /* Condenser Water SourceMdot and Parameter Set*/
  ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot CondensedWaterIn(
    redeclare package Medium = ThermoCycle.Media.StandardWater,
    Mdot_0=59.75,
    T_0=303.15)
    annotation (Placement(transformation(extent={{-100,52},{-80,72}})));
  /* Condenser Water SinkP and Parameter Set*/
  ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP1(
  redeclare package Medium =
               ThermoCycle.Media.StandardWater)
    annotation (Placement(transformation(extent={{22,52},{42,72}})));
  /* Container and Parameter Set*/
  ThermoCycle.Components.Units.Tanks.Tank_pL Container(
    redeclare package Medium = ThermoCycle.Media.R134a_CP,
    impose_L=true,
    impose_pressure=false,
    Vtot=1000,
    pstart=906000)
    annotation (Placement(transformation(extent={{-88,4},{-68,24}})));
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
        origin={-76,-20})));
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
    annotation (Placement(transformation(extent={{-44,-38},{-18,-64}})));
  /* Chilled Water SourceMdot and Parameter Set*/
  ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot ChilledWaterIn(
    redeclare package Medium = ThermoCycle.Media.StandardWater,
    Mdot_0=47.8,
    T_0=285.15)
    annotation (Placement(transformation(extent={{40,-86},{20,-66}})));
  /* Chilled Water sinkP and Parameter Set*/
  ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP2(redeclare package
      Medium = ThermoCycle.Media.StandardWater)
    annotation (Placement(transformation(extent={{-84,-68},{-104,-48}})));
  /* Compressor and Parameter Set*/
  ThermoCycle.Components.Units.ExpansionAndCompressionMachines.Compressor compressor(
  redeclare package Medium =
               ThermoCycle.Media.R134a_CP,
    V_s=1.787e-3,
    p_su_start=356000,
    p_ex_start=906000,
    T_su_start=279.15)
    annotation (Placement(transformation(extent={{42,10},{18,-14}})));
  /* Electric Drive and Parameter Set*/
  ThermoCycle.Components.Units.ExpansionAndCompressionMachines.ElectricDrive
                                                               electricDrive
    annotation (Placement(transformation(extent={{4,-12},{-16,8}})));
  /* Sensors*/
  ThermoCycle.Components.FluidFlow.Sensors.SensTp Te_out(redeclare package
      Medium =
        ThermoCycle.Media.R134a_CP)
    annotation (Placement(transformation(extent={{-2,-46},{18,-26}})));
  ThermoCycle.Components.FluidFlow.Sensors.SensTp TEA(redeclare package Medium =
        ThermoCycle.Media.R134a_CP)
    annotation (Placement(transformation(extent={{-70,-46},{-50,-26}})));
  ThermoCycle.Components.FluidFlow.Sensors.SensTp TCA(redeclare package Medium =
        ThermoCycle.Media.R134a_CP)
    annotation (Placement(transformation(extent={{-10,36},{10,56}})));
  ThermoCycle.Components.FluidFlow.Sensors.SensTp TC_out(redeclare package
      Medium =
        ThermoCycle.Media.R134a_CP)
    annotation (Placement(transformation(extent={{-70,36},{-50,56}})));
  ThermoCycle.Components.FluidFlow.Sensors.SensTp TCI(redeclare package Medium =
        ThermoCycle.Media.StandardWater)
    annotation (Placement(transformation(extent={{-64,62},{-44,82}})));
  ThermoCycle.Components.FluidFlow.Sensors.SensTp TCO(redeclare package Medium =
        ThermoCycle.Media.StandardWater)
    annotation (Placement(transformation(extent={{-18,62},{2,82}})));
  ThermoCycle.Components.FluidFlow.Sensors.SensTp TEO(redeclare package Medium =
        ThermoCycle.Media.StandardWater)
    annotation (Placement(transformation(extent={{-72,-56},{-52,-76}})));
  ThermoCycle.Components.FluidFlow.Sensors.SensTp TEI(redeclare package Medium =
        ThermoCycle.Media.StandardWater)
    annotation (Placement(transformation(extent={{-20,-56},{0,-76}})));

  /* Fault Ramps */
  Modelica.Blocks.Sources.Ramp Level_1(
    duration(displayUnit="d") = 2592000,
    startTime(displayUnit="d") = 2592000,
    offset=47.8,
    height=-4.78)
    annotation (Placement(transformation(extent={{86,-54},{66,-34}})));
  Modelica.Blocks.Sources.Ramp Level_2(
    duration(displayUnit="d") = 2592000,
    startTime(displayUnit="d") = 2592000,
    offset=47.8,
    height=-9.56)
    annotation (Placement(transformation(extent={{118,-54},{98,-34}})));
  Modelica.Blocks.Sources.Ramp Level_3(
    duration(displayUnit="d") = 2592000,
    startTime(displayUnit="d") = 2592000,
    offset=47.8,
    height=-14.34)
    annotation (Placement(transformation(extent={{86,-88},{66,-68}})));
  Modelica.Blocks.Sources.Ramp Level_4(
    duration(displayUnit="d") = 2592000,
    startTime(displayUnit="d") = 2592000,
    offset=47.8,
    height=-19.12)
    annotation (Placement(transformation(extent={{118,-88},{98,-68}})));

  /* Controller */

  Modelica.Blocks.Sources.Constant DELTAT_SP1(k=273.15 + 7)
    annotation (Placement(transformation(extent={{70,-20},{76,-14}})));
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
    annotation (Placement(transformation(extent={{88,-2},{106,-16}})));
  ThermoCycle.Components.Units.ControlSystems.SH_block sH_block(redeclare
      package Medium =
        ThermoCycle.Media.R134a_CP)
    annotation (Placement(transformation(extent={{50,46},{60,56}})));
  Modelica.Blocks.Sources.Constant DELTAT_SP(k=0.5)
    annotation (Placement(transformation(extent={{56,28},{62,34}})));
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
    annotation (Placement(transformation(extent={{72,42},{90,28}})));

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
      points={{-78,5.2},{-78,-11},{-76,-11}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(electricDrive.shaft, compressor.flange_elc) annotation (Line(
      points={{2.6,-2},{22,-2}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(Evaporator.outlet_fl1, Te_out.InFlow)
    annotation (Line(points={{-21,-46},{8,-46},{8,-45.4}},  color={0,0,255}));
  connect(valve.OutFlow, TEA.InFlow) annotation (Line(points={{-76,-29},{-76,-45.4},
          {-60,-45.4}}, color={0,0,255}));
  connect(TEA.InFlow, Evaporator.inlet_fl1) annotation (Line(points={{-60,-45.4},
          {-50,-45.4},{-50,-46},{-41,-46}}, color={0,0,255}));
  connect(Condenser.outlet_fl1, TC_out.InFlow)
    annotation (Line(points={{-41,36},{-60,36},{-60,36.6}}, color={0,0,255}));
  connect(Container.InFlow, TC_out.InFlow) annotation (Line(points={{-78,22.4},{
          -78,36.6},{-60,36.6}},  color={0,0,255}));
  connect(sinkP2.flangeB, TEO.InFlow) annotation (Line(points={{-85.6,-58},{-62,
          -58},{-62,-56.6}}, color={0,0,255}));
  connect(TEO.InFlow, Evaporator.outlet_fl2) annotation (Line(points={{-62,-56.6},
          {-52,-56.6},{-52,-56.8},{-40.8,-56.8}}, color={0,0,255}));
  connect(Evaporator.inlet_fl2, TEI.InFlow) annotation (Line(points={{-21.2,-57},
          {-15.6,-57},{-15.6,-56.6},{-10,-56.6}},
                                              color={0,0,255}));
  connect(TEI.InFlow,ChilledWaterIn. flangeB) annotation (Line(points={{-10,-56.6},
          {8,-56.6},{8,-76},{21,-76}},   color={0,0,255}));
  connect(CondensedWaterIn.flangeB, TCI.InFlow)
    annotation (Line(points={{-81,62},{-54,62},{-54,62.6}}, color={0,0,255}));
  connect(TCI.InFlow, Condenser.inlet_fl2) annotation (Line(points={{-54,62.6},{
          -48,62.6},{-48,62},{-44,62},{-44,47},{-40.8,47}}, color={0,0,255}));
  connect(sinkP1.flangeB, TCO.InFlow)
    annotation (Line(points={{23.6,62},{-8,62},{-8,62.6}}, color={0,0,255}));
  connect(Condenser.outlet_fl2, TCO.InFlow) annotation (Line(points={{-21.2,46.8},
          {-16,46.8},{-16,62.6},{-8,62.6}},
                                          color={0,0,255}));

  connect(compressor.OutFlow, TCA.InFlow) annotation (Line(points={{21.4,2},{20,
          2},{20,34},{20,36.6},{0,36.6}},  color={0,0,255}));
  connect(TCA.InFlow, Condenser.inlet_fl1) annotation (Line(points={{0,36.6},{-10,
          36.6},{-10,36},{-21,36}},color={0,0,255}));

  connect(Te_out.InFlow, compressor.InFlow) annotation (Line(points={{8,-45.4},{
          26,-45.4},{26,-46},{36.8,-46},{36.8,-9.8}},   color={0,0,255}));
  connect(DELTAT_SP1.y, PID_Compressor.SP) annotation (Line(
      points={{76.3,-17},{81.15,-17},{81.15,-11.8},{88,-11.8}},
      color={0,0,127},
      smooth=Smooth.None,
      pattern=LinePattern.Dot));
  connect(TEO.T, PID_Compressor.PV) annotation (Line(
      points={{-54,-72},{-46,-72},{-46,-92},{-38,-92},{52,-92},{52,-10},{52,-6.2},
          {88,-6.2}},
      color={0,0,127},
      pattern=LinePattern.Dot));
  connect(PID_Compressor.CS, electricDrive.f) annotation (Line(
      points={{106.54,-9},{110,-9},{110,16},{-6.4,16},{-6.4,7.4}},
      color={0,0,127},
      pattern=LinePattern.Dot));
  connect(Te_out.T, sH_block.T_measured) annotation (Line(
      points={{16,-30},{60,-30},{60,20},{42,20},{42,53.5},{49.7,53.5}},
      color={0,0,127},
      smooth=Smooth.None,
      pattern=LinePattern.Dot));
  connect(Te_out.p, sH_block.p_measured) annotation (Line(
      points={{0,-30},{-12,-30},{-12,-22},{64,-22},{64,24},{46,24},{46,49},{49.9,
          49}},
      color={0,0,127},
      smooth=Smooth.None,
      pattern=LinePattern.Dot));
  connect(DELTAT_SP.y,PID_valve. SP) annotation (Line(
      points={{62.3,31},{67.15,31},{67.15,32.2},{72,32.2}},
      color={0,0,127},
      smooth=Smooth.None,
      pattern=LinePattern.Dot));
  connect(PID_valve.CS, valve.cmd) annotation (Line(points={{90.54,35},{98,35},{
          98,34},{104,34},{104,90},{-108,90},{-108,-20},{-84,-20}},
                                                  color={0,0,127},pattern=LinePattern.Dot));
  connect(sH_block.DeltaT,PID_valve. PV) annotation (Line(points={{60.55,51.25},
          {66,51.25},{66,37.8},{72,37.8}}, color={0,0,127}));
  connect(Level_1.y, ChilledWaterIn.in_Mdot) annotation (Line(points={{65,-44},{
          46,-44},{46,-60},{36,-60},{36,-70}}, color={0,0,127}));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-140,
            -100},{140,100}})),
    experiment(StopTime=7.776e+006, Interval=600),
    __Dymola_experimentSetupOutput,
    Icon(coordinateSystem(extent={{-100,-100},{100,80}})),
              Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    uses(Modelica(version="3.2.2")));
  annotation (Documentation(info="<HTML>
    <p><big> This model allows the user to simulate the Fault Reduced Evaporator Water Flow in a Chiller system.
    The complete Chiller model is composed by the following components: compressor, evaporator, condenser, 
    one liquid container, a valve, some sensors and controllers.
    <p><big> The fault Reduced Evaporator Water Flow can be simulated by a ramp input connected to the evaporator water inlet Mdot. 
    There are four different levels, the severity is increased by level. The users can change the fault to different level or set the fault as needed.
  
  <p><big> This model is based on ThermoCycle Library (<a href=\"http://www.thermocycle.net/\">http://www.thermocycle.net/</a>) and ExternalMedia Library (<a href=\"https://github.com/modelica/ExternalMedia\">https://github.com/modelica/ExternalMedia</a>).
  It cannot be successully runned without these two Libraries loaded.
  </HTML>"));


end ReducedEvapFlow;
