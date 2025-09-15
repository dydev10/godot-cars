extends RigidBody3D

@export var wheels: Array[RaycastWheel]
@export var acceleration := 600.0
@export var maxSpeed := 20.0
@export var powerCurve: Curve
@export var tireTurnSpeed := 2.0
@export var tireMaxTurnDegree := 25.0

@export var skidMarks: Array[GPUParticles3D]

# debug state
@export var debugArrow := true
@export var debugPoint := true


var motor_input := 0
var handbrake := false
var isSlipping := false


func _unhandled_input(event: InputEvent) -> void:
	if event.is_action_pressed("handbrake"):
		handbrake = true
		isSlipping = true
	elif event.is_action_released("handbrake"):
		handbrake = false
	
	if event.is_action_pressed("move_up"):
		motor_input = 1
	elif event.is_action_released("move_up"):
		motor_input = 0

	if event.is_action_pressed("move_down"):
		motor_input = -1
	elif event.is_action_released("move_down"):
		motor_input = 0


func _physics_process(delta: float) -> void:
	_basic_steering_rotation(delta)
	
	var isGrounded := false
	var id := 0
	
	for wheel in wheels:
		if wheel.is_colliding():
			isGrounded = true
		wheel.force_raycast_update()
		_do_single_wheel_suspension(wheel)
		_do_single_wheel_acceleration(wheel)
		_do_single_wheel_traction(wheel, id)
		id += 1
	
	if isGrounded:
		center_of_mass = Vector3.ZERO
	else:
		center_of_mass_mode = RigidBody3D.CENTER_OF_MASS_MODE_CUSTOM
		center_of_mass = Vector3.DOWN * 0.5
		
	if debugPoint:
		DebugDraw3D.draw_sphere(global_position + center_of_mass, 0.2)
	if debugArrow:
		DebugDraw3D.draw_arrow_ray(global_position, linear_velocity, 1.0, Color.YELLOW, 0.02)


func _get_point_velocity(point: Vector3) -> Vector3:
	return linear_velocity + angular_velocity.cross(point - global_position)  # TODO: study this forumla to get linear velocity of a point on rigid body


func _basic_steering_rotation(delta) -> void:
	var turnInput := Input.get_axis("move_right", "move_left") * tireTurnSpeed
	
	if turnInput:
		$WheelFL.rotation.y = clamp($WheelFL.rotation.y + turnInput * delta, -deg_to_rad(tireMaxTurnDegree), deg_to_rad(tireMaxTurnDegree))
		$WheelFR.rotation.y = clamp($WheelFR.rotation.y + turnInput * delta, -deg_to_rad(tireMaxTurnDegree), deg_to_rad(tireMaxTurnDegree))
	else:
		$WheelFL.rotation.y = move_toward($WheelFL.rotation.y, 0.0, tireTurnSpeed * delta)
		$WheelFR.rotation.y = move_toward($WheelFR.rotation.y, 0.0, tireTurnSpeed * delta)


func _do_single_wheel_traction(ray: RaycastWheel, idx: int) -> void:
	if not ray.is_colliding(): return
	
	var steerSideDir := ray.global_basis.x
	var tireVel := _get_point_velocity(ray.wheel.global_position)
	var xSteeringVel := steerSideDir.dot(tireVel)
	
	var gripFactor := absf(xSteeringVel / tireVel.length())
	var xTraction := ray.gripCurve.sample_baked(gripFactor)
	
	# skid marks when drifitng or slipping
	skidMarks[idx].global_position = ray.get_collision_point() + Vector3.UP * 0.01
	skidMarks[idx].look_at(skidMarks[idx].global_position + global_basis.z)
	
	if not handbrake and gripFactor < 0.2:
		isSlipping = false
		skidMarks[idx].emitting = false
	
	# drift by nerfing xTraction when handbrake
	if handbrake:
		xTraction = 0.01
		if not skidMarks[idx].emitting:
			skidMarks[idx].emitting = true
	elif isSlipping:
		xTraction = 0.1
	
	
	## F = M * dV/T
	#var desiredAccel := (xSteeringVel * xTraction) / get_process_delta_time()
	#var xForce := -steerSideDir * desiredAccel * (mass / 4.0)  # TODO: use spring force to calculate load on each tire instead of mass/4
	
	var gravity: float = ProjectSettings.get_setting("physics/3d/default_gravity")
	var xForce := -steerSideDir * xSteeringVel * xTraction * ((mass * gravity) / 4.0)  # TODO: use spring force to calculate load on each tire instead of mass/4
	
	var fVel := -ray.global_basis.z.dot(tireVel)
	var zTraction := 0.05
	#var zForce := ray.global_basis.z * fVel * zTraction * ((mass * gravity) / 4.0)
	var zForce := global_basis.z * fVel * zTraction * ((mass * gravity) / 4.0)
	
	var forcePos := ray.wheel.global_position - global_position
	apply_force(xForce, forcePos)
	apply_force(zForce, forcePos)
	if debugArrow:
		DebugDraw3D.draw_arrow_ray(ray.wheel.global_position, xForce/mass, 1.0, Color.GREEN, 0.02)
		DebugDraw3D.draw_arrow_ray(ray.wheel.global_position, zForce/mass, 1.0, Color.PINK, 0.02)


func _do_single_wheel_acceleration(ray: RaycastWheel) -> void:
	var forwardDir := -ray.global_basis.z
	var vel := forwardDir.dot(linear_velocity)	
	ray.wheel.rotate_x((-vel * get_process_delta_time()) / ray.wheelRadius)
	
	if ray.is_colliding():
		var contact := ray.wheel.global_position
		var forcePos := contact - global_position
			
		if ray.isMotor and motor_input:
			var speedRatio := vel / maxSpeed
			var power = powerCurve.sample_baked(speedRatio)
			var forceVector: Vector3 = forwardDir * acceleration * motor_input * power
			apply_force(forceVector, forcePos)
			if debugArrow:
				DebugDraw3D.draw_arrow_ray(contact, forceVector/mass, 1.0, Color.RED, 0.02)


func _do_single_wheel_suspension(ray: RaycastWheel) -> void:
	if ray.is_colliding():
		# limit the raycast target distance
		ray.target_position.y = -(ray.restDist + ray.wheelRadius + ray.overExtend)
		
		# calculate state of spring
		var contact := ray.get_collision_point()
		var springUpDir := ray.global_transform.basis.y
		var springLen := ray.global_position.distance_to(contact) - ray.wheelRadius
		var offset := ray.restDist - springLen
		
		# adjust wheel mesh based on spring length
		ray.wheel.position.y = -springLen
		
		var springForce := ray.springStrengh * offset
		
		var worldVel := _get_point_velocity(contact)
		var relativeVel := springUpDir.dot(worldVel)
		var springDampForce := ray.springDamping * relativeVel
		
		var forceVector := (springForce - springDampForce) * ray.get_collision_normal()
		
		var forcePosOffset = ray.wheel.global_position - global_position
		apply_force(forceVector, forcePosOffset)


		# debug things
		if debugArrow:
			DebugDraw3D.draw_arrow_ray(ray.wheel.global_position, forceVector/mass, 1.0, Color.BLUE, 0.02);
