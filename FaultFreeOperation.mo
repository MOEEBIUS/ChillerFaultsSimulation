within ;
model FaultFreeOperation


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
    annotation (Placement(transformation(extent={{-14,26},{-40,52}})));

  /* Condenser Water SourceMdot and Parameter Set*/
  ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot CondensedWaterIn(
    redeclare package Medium = ThermoCycle.Media.StandardWater,
    Mdot_0=59.75,
    T_0=303.15)
    annotation (Placement(transformation(extent={{-96,50},{-76,70}})));
  /* Condenser Water SinkP and Parameter Set*/
  ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP1(
  redeclare package Medium =
               ThermoCycle.Media.StandardWater)
    annotation (Placement(transformation(extent={{26,50},{46,70}})));
  /* Container and Parameter Set*/
  ThermoCycle.Components.Units.Tanks.Tank_pL Container(
    redeclare package Medium = ThermoCycle.Media.R134a_CP,
    impose_L=true,
    impose_pressure=false,
    Vtot=1000,
    pstart=906000)
    annotation (Placement(transformation(extent={{-84,2},{-64,22}})));
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
        origin={-72,-22})));
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
    annotation (Placement(transformation(extent={{-40,-40},{-14,-66}})));
  /* Chilled Water SourceMdot and Parameter Set*/
  ThermoCycle.Components.FluidFlow.Reservoirs.SourceMdot ChilledWaterIn(
    redeclare package Medium = ThermoCycle.Media.StandardWater,
    Mdot_0=47.8,
    T_0=285.15)
    annotation (Placement(transformation(extent={{44,-88},{24,-68}})));
  /* Chilled Water sinkP and Parameter Set*/
  ThermoCycle.Components.FluidFlow.Reservoirs.SinkP sinkP2(redeclare package
      Medium = ThermoCycle.Media.StandardWater)
    annotation (Placement(transformation(extent={{-80,-70},{-100,-50}})));
  /* Compressor and Parameter Set*/
  ThermoCycle.Components.Units.ExpansionAndCompressionMachines.Compressor compressor(
  redeclare package Medium =
               ThermoCycle.Media.R134a_CP,
    V_s=1.787e-3,
    p_su_start=356000,
    p_ex_start=906000,
    T_su_start=279.15)
    annotation (Placement(transformation(extent={{46,8},{22,-16}})));
  /* Electric Drive and Parameter Set*/
  ThermoCycle.Components.Units.ExpansionAndCompressionMachines.ElectricDrive
                                                               electricDrive
    annotation (Placement(transformation(extent={{8,-14},{-12,6}})));
  /* Sensors*/
  ThermoCycle.Components.FluidFlow.Sensors.SensTp Te_out(redeclare package
      Medium =
        ThermoCycle.Media.R134a_CP)
    annotation (Placement(transformation(extent={{2,-48},{22,-28}})));
  ThermoCycle.Components.FluidFlow.Sensors.SensTp TEA(redeclare package Medium =
        ThermoCycle.Media.R134a_CP)
    annotation (Placement(transformation(extent={{-66,-48},{-46,-28}})));
  ThermoCycle.Components.FluidFlow.Sensors.SensTp TCA(redeclare package Medium =
        ThermoCycle.Media.R134a_CP)
    annotation (Placement(transformation(extent={{-6,34},{14,54}})));
  ThermoCycle.Components.FluidFlow.Sensors.SensTp TC_out(redeclare package
      Medium =
        ThermoCycle.Media.R134a_CP)
    annotation (Placement(transformation(extent={{-66,34},{-46,54}})));
  ThermoCycle.Components.FluidFlow.Sensors.SensTp TCI(redeclare package Medium =
        ThermoCycle.Media.StandardWater)
    annotation (Placement(transformation(extent={{-60,60},{-40,80}})));
  ThermoCycle.Components.FluidFlow.Sensors.SensTp TCO(redeclare package Medium =
        ThermoCycle.Media.StandardWater)
    annotation (Placement(transformation(extent={{-14,60},{6,80}})));
  ThermoCycle.Components.FluidFlow.Sensors.SensTp TEO(redeclare package Medium =
        ThermoCycle.Media.StandardWater)
    annotation (Placement(transformation(extent={{-68,-58},{-48,-78}})));
  ThermoCycle.Components.FluidFlow.Sensors.SensTp TEI(redeclare package Medium =
        ThermoCycle.Media.StandardWater)
    annotation (Placement(transformation(extent={{-16,-58},{4,-78}})));
  /* Fault Ramp */


  /* Controller */

  Modelica.Blocks.Sources.Constant DELTAT_SP1(k=273.15 + 7)
    annotation (Placement(transformation(extent={{74,-22},{80,-16}})));
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
    annotation (Placement(transformation(extent={{92,-4},{110,-18}})));
  ThermoCycle.Components.Units.ControlSystems.SH_block sH_block(redeclare
      package Medium =
        ThermoCycle.Media.R134a_CP)
    annotation (Placement(transformation(extent={{54,44},{64,54}})));
  Modelica.Blocks.Sources.Constant DELTAT_SP(k=0.5)
    annotation (Placement(transformation(extent={{60,26},{66,32}})));
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
    annotation (Placement(transformation(extent={{76,40},{94,26}})));

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
      points={{-74,3.2},{-74,-13},{-72,-13}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(electricDrive.shaft, compressor.flange_elc) annotation (Line(
      points={{6.6,-4},{26,-4}},
      color={0,0,0},
      smooth=Smooth.None));
  connect(Evaporator.outlet_fl1, Te_out.InFlow)
    annotation (Line(points={{-17,-48},{12,-48},{12,-47.4}},color={0,0,255}));
  connect(valve.OutFlow, TEA.InFlow) annotation (Line(points={{-72,-31},{-72,-47.4},
          {-56,-47.4}}, color={0,0,255}));
  connect(TEA.InFlow, Evaporator.inlet_fl1) annotation (Line(points={{-56,-47.4},
          {-46,-47.4},{-46,-48},{-37,-48}}, color={0,0,255}));
  connect(Condenser.outlet_fl1, TC_out.InFlow)
    annotation (Line(points={{-37,34},{-56,34},{-56,34.6}}, color={0,0,255}));
  connect(Container.InFlow, TC_out.InFlow) annotation (Line(points={{-74,20.4},{
          -74,34.6},{-56,34.6}},  color={0,0,255}));
  connect(sinkP2.flangeB, TEO.InFlow) annotation (Line(points={{-81.6,-60},{-58,
          -60},{-58,-58.6}}, color={0,0,255}));
  connect(TEO.InFlow, Evaporator.outlet_fl2) annotation (Line(points={{-58,-58.6},
          {-48,-58.6},{-48,-58.8},{-36.8,-58.8}}, color={0,0,255}));
  connect(Evaporator.inlet_fl2, TEI.InFlow) annotation (Line(points={{-17.2,-59},
          {-11.6,-59},{-11.6,-58.6},{-6,-58.6}},
                                              color={0,0,255}));
  connect(TEI.InFlow,ChilledWaterIn. flangeB) annotation (Line(points={{-6,-58.6},
          {12,-58.6},{12,-78},{25,-78}}, color={0,0,255}));
  connect(CondensedWaterIn.flangeB, TCI.InFlow)
    annotation (Line(points={{-77,60},{-50,60},{-50,60.6}}, color={0,0,255}));
  connect(TCI.InFlow, Condenser.inlet_fl2) annotation (Line(points={{-50,60.6},{
          -44,60.6},{-44,60},{-40,60},{-40,45},{-36.8,45}}, color={0,0,255}));
  connect(sinkP1.flangeB, TCO.InFlow)
    annotation (Line(points={{27.6,60},{-4,60},{-4,60.6}}, color={0,0,255}));
  connect(Condenser.outlet_fl2, TCO.InFlow) annotation (Line(points={{-17.2,44.8},
          {-12,44.8},{-12,60.6},{-4,60.6}},
                                          color={0,0,255}));

  connect(compressor.OutFlow, TCA.InFlow) annotation (Line(points={{25.4,0},{24,
          0},{24,32},{24,34.6},{4,34.6}},  color={0,0,255}));
  connect(TCA.InFlow, Condenser.inlet_fl1) annotation (Line(points={{4,34.6},{-6,
          34.6},{-6,34},{-17,34}}, color={0,0,255}));

  connect(Te_out.InFlow, compressor.InFlow) annotation (Line(points={{12,-47.4},
          {30,-47.4},{30,-48},{40.8,-48},{40.8,-11.8}}, color={0,0,255}));
  connect(DELTAT_SP1.y, PID_Compressor.SP) annotation (Line(
      points={{80.3,-19},{85.15,-19},{85.15,-13.8},{92,-13.8}},
      color={0,0,127},
      smooth=Smooth.None,
      pattern=LinePattern.Dot));
  connect(TEO.T, PID_Compressor.PV) annotation (Line(
      points={{-50,-74},{-42,-74},{-42,-94},{-34,-94},{56,-94},{56,-12},{56,-8.2},
          {92,-8.2}},
      color={0,0,127},
      pattern=LinePattern.Dot));
  connect(PID_Compressor.CS, electricDrive.f) annotation (Line(
      points={{110.54,-11},{114,-11},{114,14},{-2.4,14},{-2.4,5.4}},
      color={0,0,127},
      pattern=LinePattern.Dot));
  connect(Te_out.T, sH_block.T_measured) annotation (Line(
      points={{20,-32},{64,-32},{64,18},{46,18},{46,51.5},{53.7,51.5}},
      color={0,0,127},
      smooth=Smooth.None,
      pattern=LinePattern.Dot));
  connect(Te_out.p, sH_block.p_measured) annotation (Line(
      points={{4,-32},{-8,-32},{-8,-24},{68,-24},{68,22},{50,22},{50,47},{53.9,47}},
      color={0,0,127},
      smooth=Smooth.None,
      pattern=LinePattern.Dot));
  connect(DELTAT_SP.y,PID_valve. SP) annotation (Line(
      points={{66.3,29},{71.15,29},{71.15,30.2},{76,30.2}},
      color={0,0,127},
      smooth=Smooth.None,
      pattern=LinePattern.Dot));
  connect(PID_valve.CS, valve.cmd) annotation (Line(points={{94.54,33},{102,33},
          {102,32},{108,32},{108,88},{-104,88},{-104,-22},{-80,-22}},
                                                  color={0,0,127},pattern=LinePattern.Dot));
  connect(sH_block.DeltaT,PID_valve. PV) annotation (Line(points={{64.55,49.25},
          {70,49.25},{70,35.8},{76,35.8}}, color={0,0,127}));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-140,
            -100},{140,100}})),
    experiment(StopTime=7.776e+006, Interval=600),
    __Dymola_experimentSetupOutput,
    Icon(coordinateSystem(extent={{-100,-100},{100,80}})),
              Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    uses(Modelica(version="3.2.2")));
  annotation (Documentation(info="<HTML>
  <p><big> This model allows the user to simulate the Fault Free Operation in a Chiller system.
    The complete Chiller model is composed by the following components: compressor, evaporator, condenser, 
    one liquid container, a valve, some sensors and controllers.

  
  <p><big> This model is based on ThermoCycle Library (<a href=\"http://www.thermocycle.net/\">http://www.thermocycle.net/</a>) and ExternalMedia Library (<a href=\"https://github.com/modelica/ExternalMedia\">https://github.com/modelica/ExternalMedia</a>).
  It cannot be successully runned without these two Libraries loaded.
  </HTML>"));


end FaultFreeOperation;
