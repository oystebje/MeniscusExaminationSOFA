#*************************************************
# Scene by Øystein Bjelland, Dept. of ICT and Natural Sciences, NTNU Ålesund
# Email: oystein.bjelland@ntnu.no

#Units
#Mass [metric tonne (or Mg)], length [mm], time [s], force [N], Stress [kPa], density [tonne/mm^3], Young's modulus [MPa], Poisson's ratio [-], gravity [mm/s^2], stiffness [N/mm]

#*************************************************

import Sofa.Core

# Required import for python
import Sofa
import SofaRuntime


#OPTIONS BEGIN******************************************************************************************

# Select  anatomy to include
addLateralMeniscus = True

#OPTIONS END******************************************************************************************

# Function called when the scene graph is being created
def createScene(rootNode):
     
    rootNode.dt=0.01 
    rootNode.gravity=[0, -9.81e3, 0] 

    confignode = rootNode.addChild("Config")
    confignode.addObject('RequiredPlugin', name="Sofa.Component.Collision.Geometry")
    confignode.addObject('RequiredPlugin', name="SofaPython3", printLog=False)
    confignode.addObject('OglSceneFrame', style="Arrows", alignment="TopRight")
    
    rootNode.addObject('DefaultVisualManagerLoop')
    rootNode.addObject('DefaultPipeline', name="pipeline", depth="6", verbose="0")
    rootNode.addObject('BruteForceBroadPhase')
    rootNode.addObject('BVHNarrowPhase')
    rootNode.addObject('DefaultContactManager', name="response", response="FrictionContactConstraint") #responseParams="mu=0.1") #Does this call on the penalty method, instead of Lagrange M?
    rootNode.addObject('LocalMinDistance', alarmDistance="3.408", contactDistance="1.136", angleCone="0.0")
    rootNode.addObject('FreeMotionAnimationLoop') #, parallelCollisionDetectionAndFreeMotion = True, parallelODESolving = True)
    LCP = rootNode.addObject('LCPConstraintSolver', tolerance="0.001", maxIt="1000", computeConstraintForces="True", mu="0.000001")
    #LCP = rootNode.addObject('GenericConstraintSolver', name="GCS", maxIt=1000, tolerance=0.001, computeConstraintForces=True)

    #Camera
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Visual')
    rootNode.addObject('RequiredPlugin', name='Sofa.GL.Component.Rendering3D')
    rootNode.addObject('InteractiveCamera',name='camera') #, position='0 0 500', orientation='0 0 0', fovy='60', znear='1', zfar='1000') 
    rootNode.camera.position.value = [-150,30,0]

    #Haptics
    rootNode.addObject('RequiredPlugin', name='Geomagic')
    driver = rootNode.addObject('GeomagicDriver', name='GeomagicDevice', deviceName="Default Device", scale="10", drawDeviceFrame="1", positionBase="0 0 0", drawDevice="0", orientationBase="0 0.707 0 -0.707", maxInputForceFeedback = "3")  #scale attribute only affects position
    
    #LATERAL MENISCUS NODE BEGIN*************************************************************************************************************************************************************************
    if addLateralMeniscus == True:
        lateralMeniscus = rootNode.addChild('lateralMeniscus Hyperelastic')
        lateralMeniscus.gravity = [0, -9.81e3, 0]
        lateralMeniscus.addObject('EulerImplicitSolver', name="cg_odesolver")
        #lateralMeniscus.addObject('CGLinearSolver', name="linear solver", iterations="25", tolerance="1e-09", threshold="1e-09")
        lateralMeniscus.addObject('ConstantSparsityPatternSystem', template="CompressedRowSparseMatrixd", name="A")
        lateralMeniscus.addObject('SparseLDLSolver', name="linear solver", template="CompressedRowSparseMatrixd")
        
        lateralMeniscus.addObject('MeshGmshLoader', name="meshLoader", filename="mesh/lateralMeniscusComputationModelSTP.msh", rotation="90 270 -182", translation="150 -31.5 -26", scale3d="26 26 26")
        lateralMeniscus.addObject('TetrahedronSetTopologyContainer', name="topo", position="@meshLoader.position", tetrahedra="@meshLoader.tetrahedra")
        lateralMeniscusDofs = lateralMeniscus.addObject('MechanicalObject', name="lateralMeniscusDofs", src="@meshLoader")
        lateralMeniscus.addObject('TetrahedronSetGeometryAlgorithms', template="Vec3d", name="GeomAlgo")
        lateralMeniscus.addObject('DiagonalMass', name="mass", massDensity="1.09e-9", topology="@topo", geometryState="@lateralMeniscusDofs") # massDensity="10.9e-03"
        
        #lateralMeniscus.addObject('TetrahedralCorotationalFEMForceField', template="Vec3d", name="FEM", poissonRatio="0.29", youngModulus="4.24") # 3.84
        #lateralMeniscus.addObject('TetrahedronHyperelasticityFEMForceField', name='HyperelasticMaterial', materialName ='StVenantKirchhoff', ParameterSet="2.8846 1.9231") #ParameterSet="3448.2759 31034.483")
        #lateralMeniscus.addObject('TetrahedronHyperelasticityFEMForceField', name='HyperelasticMaterial', materialName ='NeoHookean', ParameterSet="1.16 2.38")
        lateralMeniscus.addObject('TetrahedronHyperelasticityFEMForceField', name='HyperelasticMaterial', materialName ='StableNeoHookean', ParameterSet="1.16 2.38")
        #lateralMeniscus.addObject('PrecomputedConstraintCorrection', recompute="true")
        #lateralMeniscus.addObject('UncoupledConstraintCorrection')
        lateralMeniscus.addObject('LinearSolverConstraintCorrection')
        
        lateralMeniscus.addObject('FixedConstraint', name="FixedConstraint", indices="1 2 5 6 7 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 42 43 44 45 46 47 48 49 50 51 52 53 54 55 56 57 58 59 60 61 62 63 64 65 66 67 68 69 70 71 72 73 73 197 198 199 200 201 202 203 204 205 206 207 208 209 210 211 212 213 214 215 216 217 218 219 220 221 222 223 224 225 226 227 228 229 230 231 232 233 234 235 236 237 238 239 240 241 242 243 244 245 246 247 248 249 251 252 253 254 255 256 257")

        #Lateral meniscus visual model
        lateralMeniscusVisual = lateralMeniscus.addChild('Lateral Meniscus Visual Model')
        lateralMeniscusVisual.addObject('MeshOBJLoader', name="meshLoader_LM", filename="mesh/LATERAL_MENISCUS_VISUAL_MODEL.obj", rotation="-85 277 -185", translation="270 105 -80", scale3d="0.0545 0.0545 0.0545", handleSeams="1")
        lateralMeniscusVisual.addObject('OglModel', name="VisualModel", src="@meshLoader_LM", color="1.0 0.2 0.2 1.0")
        lateralMeniscusVisual.addObject('BarycentricMapping', name="visual mapping", input="@../lateralMeniscusDofs", output="@VisualModel")

        #Lateral meniscus collision model (extracts triangular mesh from computation model)
        lateralMeniscusTriangularSurface = lateralMeniscus.addChild('Lateral Meniscus Triangular Surface Model')
        lateralMeniscusTriangularSurface.addObject('TriangleSetTopologyContainer', name="Container")
        lateralMeniscusTriangularSurface.addObject('TriangleSetTopologyModifier', name="Modifier")
        lateralMeniscusTriangularSurface.addObject('Tetra2TriangleTopologicalMapping', input="@../topo", output="@Container")
        lateralMeniscusTriangularSurface.addObject('TriangleCollisionModel', name="CollisionModel", contactStiffness="0.13")
    #LATERAL MENISCUS NODE END***************************************************************************************************************************************************************************

    #LATERAL MENISCUS NODE BEGIN*************************************************************************************************************************************************************************
    if addLateralMeniscus == True:
        lateralMeniscus2 = rootNode.addChild('lateralMeniscus2')
        lateralMeniscus2.gravity = [0, -9.81e3, 0]
        lateralMeniscus2.addObject('EulerImplicitSolver', name="cg_odesolver")
        lateralMeniscus2.addObject('CGLinearSolver', name="linear solver", iterations="25", tolerance="1e-09", threshold="1e-09")
        lateralMeniscus2.addObject('MeshGmshLoader', name="meshLoader", filename="mesh/lateralMeniscusComputationModelSTP.msh", rotation="90 270 -182", translation="150 -31.5 34", scale3d="26 26 26")
        #lateralMeniscus2.addObject('TetrahedronSetTopologyContainer', name="topo", position="-0.5 0 0    0.542 0.455 0.542 0.455",  src="@meshLoader")
        lateralMeniscus2.addObject('TetrahedronSetTopologyContainer', name="topo", position="@meshLoader.position", tetrahedra="@meshLoader.tetrahedra")
        lateralMeniscusDofs2 = lateralMeniscus2.addObject('MechanicalObject', name="lateralMeniscusDofs", src="@meshLoader")
        lateralMeniscus2.addObject('TetrahedronSetGeometryAlgorithms', template="Vec3d", name="GeomAlgo")
        lateralMeniscus2.addObject('DiagonalMass', name="mass", massDensity="1.09e-9", topology="@topo", geometryState="@lateralMeniscusDofs") # massDensity="10.9e-03"
        lateralMeniscus2.addObject('TetrahedralCorotationalFEMForceField', template="Vec3d", name="FEM", poissonRatio="0.29", youngModulus="3.85") # 3.84
        lateralMeniscus2.addObject('PrecomputedConstraintCorrection', recompute="true")
        lateralMeniscus2.addObject('FixedConstraint', name="FixedConstraint", indices="1 2 5 6 7 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 42 43 44 45 46 47 48 49 50 51 52 53 54 55 56 57 58 59 60 61 62 63 64 65 66 67 68 69 70 71 72 73 73 197 198 199 200 201 202 203 204 205 206 207 208 209 210 211 212 213 214 215 216 217 218 219 220 221 222 223 224 225 226 227 228 229 230 231 232 233 234 235 236 237 238 239 240 241 242 243 244 245 246 247 248 249 251 252 253 254 255 256 257")

        #Lateral meniscus visual model
        lateralMeniscusVisual2 = lateralMeniscus2.addChild('Lateral Meniscus 2 Visual Model')
        lateralMeniscusVisual2.addObject('MeshOBJLoader', name="meshLoader_LM", filename="mesh/LATERAL_MENISCUS_VISUAL_MODEL.obj", rotation="-85 277 -185", translation="270 105 -20", scale3d="0.0545 0.0545 0.0545", handleSeams="1")
        lateralMeniscusVisual2.addObject('OglModel', name="VisualModel", src="@meshLoader_LM", color="1.0 0.2 0.2 1.0")
        lateralMeniscusVisual2.addObject('BarycentricMapping', name="visual mapping", input="@../lateralMeniscusDofs", output="@VisualModel")

        #Lateral meniscus collision model (extracts triangular mesh from computation model)
        lateralMeniscusTriangularSurface2 = lateralMeniscus2.addChild('Lateral Meniscus Triangular Surface Model')
        lateralMeniscusTriangularSurface2.addObject('TriangleSetTopologyContainer', name="Container")
        lateralMeniscusTriangularSurface2.addObject('TriangleSetTopologyModifier', name="Modifier")
        lateralMeniscusTriangularSurface2.addObject('Tetra2TriangleTopologicalMapping', input="@../topo", output="@Container")
        lateralMeniscusTriangularSurface2.addObject('TriangleCollisionModel', name="CollisionModel", contactStiffness="0.13")
    #LATERAL MENISCUS NODE END***************************************************************************************************************************************************************************


    #OMNI NODE BEGIN*************************************************************************************************************************************************************************************
    omni = rootNode.addChild('Omni')
    omniDOFs = omni.addObject('MechanicalObject', name='DOFs', template='Rigid3d', position='@GeomagicDevice.positionDevice')
    #OMNI NODE END*********************************************************************************************************************************************************************************************

    #INSTRUMENT NODE BEGIN*************************************************************************************************************************************************************************************
    instrument = rootNode.addChild('Instrument')
    instrument.addObject('EulerImplicitSolver', name="ODE solver") 
    instrument.addObject('SparseLDLSolver', template='CompressedRowSparseMatrixMat3x3d')
    #instrument.addObject('CGLinearSolver', name="linear solver", iterations="25", tolerance="1.0e-5", threshold="10.0e-5")
    instrument.addObject('MechanicalObject', name='instrumentState', template='Rigid3d')
    instrument.addObject('UniformMass', name="mass", totalMass="3.5e-05") #[kg] 3.5e-05 Mass of instrument measured to be 35 g. 
    instrument.addObject('LCPForceFeedback', activate="true", forceCoef="1.0") #Multiply haptic force by this coefficient to scale force in haptic device. 
    #instrument.addObject('UncoupledConstraintCorrection')
    instrument.addObject('LinearSolverConstraintCorrection')
    instrument.addObject('RestShapeSpringsForceField', stiffness="1000000", angularStiffness="1000000", external_rest_shape="@/Omni", points="0")

            
    #Instrument visual model
    instrumentVisu = instrument.addChild('InstrumentVisualModel')
    instrumentVisu.addObject('MeshOBJLoader', name="loader", filename="mesh/ARTHREX_AR_10000_HookProbe.obj", scale3d="1 1 1", handleSeams="1")
    instrumentVisu.addObject('OglModel', name="InstrumentVisualmodel", src="@loader", color="gray", rx="270", ry="90", rz="90", dy="5", dz="0")
    instrumentVisu.addObject('RigidMapping', name="MM->VM mapping", input="@instrumentState", output="@InstrumentVisualModel") 
            
    #Instrument collision model
    instrumentCollision = instrument.addChild('instrumentCollision')
    instrumentCollision.addObject('MeshOBJLoader', name="loader", filename="mesh/ARTHREX_AR_10000_HookProbe.obj", scale3d="1 1 1", handleSeams="1")
    instrumentCollision.addObject('MeshTopology', src="@loader")
    instrumentCollision.addObject('MechanicalObject', name="instrumentCollisionState1", src="@loader",rx="270", ry="90", rz="90", dy="5", dz="0")
    instrumentCollision.addObject('LineCollisionModel', contactStiffness="0.44e-20") 
    instrumentCollision.addObject('PointCollisionModel', contactStiffness="0.44e-20") 
    instrumentCollision.addObject('RigidMapping', name="MM->CM mapping", input="@instrumentState", output="@instrumentCollisionState1")
    #INSTRUMENT NODE END**********************************************************************************************************************************************************************************************

    
    return rootNode
