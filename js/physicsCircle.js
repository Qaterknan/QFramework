function PhysicsCircle( options ){
	Object2D.call(this, options);
	
	this.color = options.color === undefined ? "#000" : options.color;
	this.radius = options.radius === undefined ? 10 : options.radius;
	// Težiště vzhledem ke středu renderování
	this.centerOfMass = options.centerOfMass === undefined ? new Vec2(0,0) : options.centerOfMass;
	if(!this.density)
		this.mass = options.mass === undefined ? 1 : options.mass;
	else{
		this.mass = this.density*Math.PI*this.radius*this.radius;
	}
	
	this.inertiaMoment = options.inertiaMoment === undefined ? this.mass*this.radius*this.radius/2 : options.inertiaMoment;
	
	this.forces = []; // Formát je objekt s vlastnostmi point(Vec2d) a force(Vec2d)
	this.colliding = true;
	
	this.collisionPoints = [];
};
PhysicsCircle.prototype = Object.create( Object2D.prototype );

PhysicsCircle.prototype.render = function(ctx){
	if(!this.texture){
		ctx.save();
		ctx.translate(this.position.x,this.position.y);
		ctx.rotate(this.rotation);
		ctx.fillStyle = this.color;
		ctx.beginPath();
		ctx.arc(0,0,this.radius,0,Math.PI);
		ctx.closePath();
		ctx.fill();
		ctx.fillStyle = "#f00";
		ctx.beginPath();
		ctx.arc(0,0,this.radius,Math.PI,Math.PI*2);
		ctx.closePath();
		ctx.fill()
		ctx.restore();
	}
	Object2D.prototype.render.call(this, ctx);
	if(this.forces.length){
		ctx.save();
		ctx.translate(this.position.x, this.position.y);
		var v;
		for(var i = 0; i < this.forces.length; i++){
			v = this.forces[i].point.clone().rotate(this.rotation);
			ctx.beginPath();
			ctx.arc(v.x,v.y,5,0,Math.PI*2);
			ctx.stroke();
			ctx.closePath();
			if(this.forces[i].force.x == 0 && this.forces[i].force.y == 0)
				continue;
			ctx.beginPath();
			ctx.moveTo(v.x,v.y);
			ctx.lineTo(this.forces[i].force.x,this.forces[i].force.y);
			ctx.stroke();
			ctx.closePath();
		};
		ctx.restore();
	}
};
PhysicsCircle.prototype.stopMotion = function(){
	this.angularVelocity = 0;
	this.acceleration.set(0,0);
	this.velocity.set(0,0);
};
PhysicsCircle.prototype.applyForce = function(point,force){
	var toPoint = new Vec2().subVectors(point, this.centerOfMass).rotate(this.rotation);
	var projekce = force.getProjections(toPoint);
	this.acceleration.add(projekce[0].divideScalar(this.mass));
	this.angularAcceleration += (toPoint.cross(projekce[1]))/this.inertiaMoment;
};

PhysicsCircle.prototype.tick = function(dt){
	for(var i = this.forces.length-1;this.forces[i];i--){
		this.applyForce(this.forces[i].point,this.forces[i].force);
		if(!this.forces[i].permanent){
			this.forces.splice(i,1);
		}
	};
	Object2D.prototype.tick.call(this,dt);
};

PhysicsCircle.prototype.checkRectangleCollision = function(object, dt){
	if(this == object)
		return false;
	if(object instanceof PhysicsCircle){
		if(this.position.distanceToSquared(object.position) > (this.radius+object.radius)*(this.radius+object.radius))
			return false;
		console.log("collision");
		var point = this.position.clone().sub(object.position).normalize().multiplyScalar(object.radius).add(object.position);
		this.collisionPoints.push(point);
		return true;
	}
};

PhysicsCircle.prototype.onCollision = function (object, dt){
	
	if(!this.collisionPoints.length)
		return;
	//this.forces = [];
	var velThis, velObject, angVelThis, angVelObject, pointVelThis, pointVelObject, relVel, pointToThis, pointToObject, normal, projections, harmonicMass, harmonicInertia, pointLengthThis, pointLengthObject;
	for(var i = 0; i < this.collisionPoints.length; i++){
		
		pointToThis = new Vec2().subVectors(this.collisionPoints[i], this.position);
		pointToObject = new Vec2().subVectors(this.collisionPoints[i], object.position);
		
		velThis = this.velocity;
		velObject = object.velocity;
		angVelThis = this.angularVelocity;
		angVelObject = object.angularVelocity;
		
		pointVelThis = new Vec2(-angVelThis*pointToThis.y + velThis.x, angVelThis*pointToThis.x + velThis.y);
		pointVelObject = new Vec2(-angVelObject*pointToObject.y + velObject.x, angVelObject*pointToObject.x + velObject.y);
		relVel =pointVelThis.sub(pointVelObject);
		
		normal = object.getSideNormal(this.collisionPoints[i]);
		projections = relVel.getProjections(normal);
		
		pointLengthThis = pointToThis.length();
		pointLengthObject = pointToObject.length();
		
		harmonicMass = 2*this.mass*object.mass/(this.mass+object.mass);
		harmonicInertia = 2*this.inertiaMoment*object.inertiaMoment/(this.inertiaMoment*pointLengthThis+object.inertiaMoment*pointLengthObject);
		
		projections[0].multiplyScalar(-harmonicMass/dt);
		this.forces.push({
			force : projections[0].clone(),
			point : new Vec2(),
		});
		object.forces.push({
			force : projections[0].multiplyScalar(-1),
			point : new Vec2(),
		});
		
		projections[1].multiplyScalar(-harmonicInertia/dt);
		pointToThis.rotate(-this.rotation);
		pointToObject.rotate(-object.rotation);
		this.forces.push({
			force : projections[1].clone().divideScalar(pointLengthThis),
			point : pointToThis,
		});
		object.forces.push({
			force : projections[1].divideScalar(-pointLengthObject),
			point : pointToObject,
		});
	};
	this.rotation -= this.angularVelocity*dt;
	this.position.sub( this.velocity.clone().multiplyScalar(dt) );
	this.collisionPoints = [];
};

PhysicsCircle.prototype.getSideNormal = function (point){
	var u = -this.rotation;
	var toPoint = point.clone().sub(this.position).rotate(u);
	return new Vec2(-toPoint.y,toPoint.x).rotate(-u);
};