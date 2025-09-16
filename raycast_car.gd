extends RigidBody3D
class_name RaycastCar

@export var wheels: Array[RaycastWheel]
@export var acceleration := 600.0
@export var maxSpeed := 20.0
@export var powerCurve: Curve
@export var tireTurnSpeed := 2.0
@export var tireMaxTurnDegree := 25.0

@export var skidMarks: Array[GPUParticles3D]

# debug state
@export var showDebug := false


@onready var totalWheels := wheels.size()

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
	var isGrounded := false
	var id := 0
	
	for wheel in wheels:
		_basic_steering_rotation(wheel, delta)
		wheel._apply_wheel_physics(self)
		
		# skid marks when drifitng or slipping
		skidMarks[id].global_position = wheel.get_collision_point() + Vector3.UP * 0.01
		skidMarks[id].look_at(skidMarks[id].global_position + global_basis.z)
		
		if not handbrake and wheel.gripFactor < 0.2:
			isSlipping = false
			skidMarks[id].emitting = false
	
		if handbrake and not skidMarks[id].emitting:
			skidMarks[id].emitting = true
		
		if wheel.is_colliding():
			isGrounded = true
		
		id += 1
	
	if isGrounded:
		center_of_mass = Vector3.ZERO
	else:
		center_of_mass_mode = RigidBody3D.CENTER_OF_MASS_MODE_CUSTOM
		center_of_mass = Vector3.DOWN * 0.5
		
	if showDebug:
		DebugDraw3D.draw_sphere(global_position + center_of_mass, 0.2)
		DebugDraw3D.draw_arrow_ray(global_position, linear_velocity, 1.0, Color.GREEN, 0.02)


func _get_point_velocity(point: Vector3) -> Vector3:
	return linear_velocity + angular_velocity.cross(point - global_position)  # TODO: study this forumla to get linear velocity of a point on rigid body


func _basic_steering_rotation(wheel: RaycastWheel, delta: float) -> void:
	if not wheel.isSteer: return
	
	var turnInput := Input.get_axis("move_right", "move_left") * tireTurnSpeed
	if turnInput:
		wheel.rotation.y = clamp(wheel.rotation.y + turnInput * delta, -deg_to_rad(tireMaxTurnDegree), deg_to_rad(tireMaxTurnDegree))
	else:
		wheel.rotation.y = move_toward(wheel.rotation.y, 0.0, tireTurnSpeed * delta)
