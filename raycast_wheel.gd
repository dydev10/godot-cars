extends RayCast3D
class_name RaycastWheel

@export_group("Suspension")
@export var springStrengh := 100
@export var springDamping := 2.0  # damp coefficient is decided by: d=z(2Sqrt[k*m]) eg: for z=0.1,m=50,k=5000, d-> 31.6228
@export var restDist := 0.5
@export var wheelRadius := 0.4
@export var overExtend := 0.0

@export_group("Motor & Traction")
@export var isMotor := false
@export var isSteer := false
@export var gripCurve: Curve

@export_group("Debug Wheel")
@export var showDebug := false

@onready var wheel: Node3D = get_child(0)  # assumes mesh is the first child node

# global var so that it can be used by RaycastCar class
var gripFactor := 0.0

func _ready() -> void:
	# set the raycast target to account for wheel size and allowed over extension 
	target_position.y = -(restDist + wheelRadius + overExtend)


func _apply_wheel_physics(car: RaycastCar) -> void:
	# need to force raycast to update before other calculations in this physics step
	force_raycast_update()
	
	## Get forward fdirection and velocity of wheel/raycast 
	var forwardDir := -global_basis.z
	var vel := forwardDir.dot(car.linear_velocity)
	
	## Rotate wheel mesh
	wheel.rotate_x((-vel * get_process_delta_time()) / wheelRadius)
	
	# no forces added by this wheel if it is not on ground
	if not is_colliding(): return
	
	# wheel is on the ground, calculate all forces and apply on car body
	
	## Calculate the spring length based on raycast hit distance
	var rayHit := get_collision_point()
	var springLen := maxf(0.0, global_position.distance_to(rayHit) - wheelRadius)
	var offset := restDist - springLen
	
	## Adjust local y position of wheel mesh based on spring length
	wheel.position.y = -springLen  # TODO: lerp the position of wheel to avoid jumps on sudden ground height change

	## Set contact point as the wheel origin point and calculate local force position on car body
	var contact := wheel.global_position
	var forcePos := contact - car.global_position
	
	##
	#
	## Spring forces
	var springForce := springStrengh * offset
	var tireVel := car._get_point_velocity(contact)
	var springUpDir := global_basis.y
	var springVel := springUpDir.dot(tireVel)
	var springDampForce := springDamping * springVel
	
	## Final suspension force (yForce)
	var yForce: Vector3 = (springForce - springDampForce) * get_collision_normal()
	
	##
	#
	## Acceleration
	var accelForce := Vector3.ZERO
	if isMotor and car.motor_input:
		var speedRatio := vel / car.maxSpeed
		var power = car.powerCurve.sample_baked(speedRatio)
		accelForce = forwardDir * car.acceleration * car.motor_input * power
	
	##
	#
	## Tire X Traction (Steering)
	var steerSideDir := global_basis.x
	var xSteeringVel := steerSideDir.dot(tireVel)
	
	## Get dynamic tire grip from curve based on slip ratio
	gripFactor = absf(xSteeringVel / tireVel.length())
	var xTraction := gripCurve.sample_baked(gripFactor)

	## Stop drifting if tire has enough grip and no handbrake input
	if not car.handbrake and gripFactor < 0.2:
		car.isSlipping = false
	
	## Drift by nerfing xTraction when handbrake or keep drifting if in drift state but with increased traction when no handbrake
	if car.handbrake:
		xTraction = 0.01
	elif car.isSlipping:
		xTraction = 0.1
	
	## Get final gravity acting on car body
	var gravity = -car.get_gravity().y
	
	## Final Steer force (xForce)
	var xForce: Vector3 = -steerSideDir * xSteeringVel * xTraction * ((car.mass * gravity) / car.totalWheels)  # TODO: use spring force to calculate load on each tire instead of mass/totalWheels

	##
	#
	## Tire Z Traction (Drag)
	var fVel := forwardDir.dot(tireVel)
	var zTraction := 0.05
	
	## Final tire drag force (zForce)
	var zForce: Vector3 = global_basis.z * fVel * zTraction * ((car.mass * gravity) / car.totalWheels)
	
	##
	#
	## Apply forces to car body
	car.apply_force(yForce, forcePos)      # suspension
	car.apply_force(xForce, forcePos)      # steer
	car.apply_force(zForce, forcePos)      # drag
	car.apply_force(accelForce, forcePos)  #acceleration
	
	##
	#
	## Debug Force Arrows
	if showDebug:
		DebugDraw3D.draw_arrow_ray(contact, yForce/car.mass, 1.0, Color.BLUE, 0.02)
		DebugDraw3D.draw_arrow_ray(contact, xForce/car.mass, 1.0, Color.YELLOW, 0.02)
		DebugDraw3D.draw_arrow_ray(contact, zForce/car.mass, 1.0, Color.PINK, 0.02)
		DebugDraw3D.draw_arrow_ray(contact, accelForce/car.mass, 1.0, Color.RED, 0.02)
