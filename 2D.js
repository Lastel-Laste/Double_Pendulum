// 2D.js

var world;
var root, edge, edge2, end;
var positions = []; // 과거 위치 배열
var trailLength = 1000; // 잔상 길이
var trailOpacity = 0.2; // 잔상 투명도

function init() {
    var   b2Vec2 = Box2D.Common.Math.b2Vec2
    ,	  b2BodyDef = Box2D.Dynamics.b2BodyDef
    ,	  b2Body = Box2D.Dynamics.b2Body
    ,	  b2FixtureDef = Box2D.Dynamics.b2FixtureDef
    ,	  b2Fixture = Box2D.Dynamics.b2Fixture
    ,	  b2World = Box2D.Dynamics.b2World
    ,	  b2MassData = Box2D.Collision.Shapes.b2MassData
    ,	  b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape
    ,	  b2CircleShape = Box2D.Collision.Shapes.b2CircleShape
    ,     b2RevoluteJointDef = Box2D.Dynamics.Joints.b2RevoluteJointDef
    ,     b2WeldJointDef = Box2D.Dynamics.Joints.b2WeldJointDef
    ,	  b2DebugDraw = Box2D.Dynamics.b2DebugDraw
    ,     canvas = document.getElementById("canvas")
    ,     ctx = canvas.getContext("2d")
    ;
    
    world = new b2World(
        new b2Vec2(0, 9.8)    //gravity
    ,  true                 //allow sleep
    );
    
    var fixDef  = new b2FixtureDef
    ,   bodyDef = new b2BodyDef
    ,   groundWidth = window.innerWidth
    ,   groundHeight = window.innerHeight
    ;   fixDef.density = 1.0
    ;   fixDef.friction = 0.0
    ;   fixDef.restitution = 0.0
    ;   fixDef.filter.maskBits = 0
    ;

    // StaticBody
    bodyDef.type = b2Body.b2_staticBody;

    root = createCircle(groundWidth / 2, groundHeight / 2, 5, fixDef, bodyDef, b2Vec2, b2CircleShape);

    // DynamicBody
    bodyDef.type = b2Body.b2_dynamicBody;
    
    edge = createPolygon(groundWidth / 2 + 100, groundHeight / 2, 4, 100, 3, fixDef, bodyDef, b2Vec2, b2PolygonShape, 0, 0, 0)
    , edgeLength = 100 // The length of the edge
    , edgeAngle = edge.GetAngle() // The current angle of the edge
    , otherEndX = edge.GetPosition().x + edgeLength * Math.cos(edgeAngle)
    , otherEndY = edge.GetPosition().y + edgeLength * Math.sin(edgeAngle)
    ;
    
    edge2 = createPolygon(otherEndX+100, otherEndY, 4, 100, 3, fixDef, bodyDef, b2Vec2, b2PolygonShape, 0, 0, 0)
    , edge2Length = 100 // The length of the edge
    , edge2Angle = edge2.GetAngle() // The current angle of the edge
    , otherEndX2 = edge2.GetPosition().x + edge2Length * Math.cos(edge2Angle)
    , otherEndY2 = edge2.GetPosition().y + edge2Length * Math.sin(edge2Angle)
    ;
    end =  createCircle(otherEndX2, otherEndY2, 5, fixDef, bodyDef, b2Vec2, b2CircleShape);
    
    var jointAnchorRoot = root.GetWorldCenter();
    createRevoluteJoint(root, edge, jointAnchorRoot, b2RevoluteJointDef);
    var jointAnchorEdge = new b2Vec2(otherEndX, otherEndY);
    createRevoluteJoint(edge, edge2, jointAnchorEdge, b2RevoluteJointDef);
    var jointAnchorEdge2 = new b2Vec2(otherEndX2, otherEndY2);
    createWeldJoint(edge2, end, jointAnchorEdge2, b2WeldJointDef);

    window.setInterval(update, 1000 / 240);
};

function createWall(x, y, width, height, bodyDef, b2Body, b2FixtureDef, b2PolygonShape) {
    bodyDef.type = b2Body.b2_staticBody;
    bodyDef.position.x = x;
    bodyDef.position.y = y;

    var wall = new b2FixtureDef;
    wall.density = 1.0;
    wall.friction = 0.5;
    wall.restitution = 0.2;
    wall.shape = new b2PolygonShape;
    wall.shape.SetAsBox(width, height);

    world.CreateBody(bodyDef).CreateFixture(wall);
}

function createCircle(x, y, size, fixDef, bodyDef, b2Vec2, b2CircleShape) {
    fixDef.shape = new b2CircleShape (size);
    bodyDef.position.x = x;
    bodyDef.position.y = y;

    var objBody = world.CreateBody(bodyDef);
    objBody.CreateFixture(fixDef);
    
    var initialVelocityX = (Math.random() * 100) - 50; // Range: -5 to 5
    var initialVelocityY = (Math.random() * 100) - 50; // Range: -5 to 5
    objBody.SetLinearVelocity(new b2Vec2(initialVelocityX, initialVelocityY));

    var initialAngularVelocity = (Math.random() * 10) - 5; // Range: -5 to 5
    objBody.SetAngularVelocity(initialAngularVelocity);

    return objBody;
}

function createPolygon(x, y, n, Width, Height, fixDef, bodyDef, b2Vec2, b2PolygonShape, vx, vy, av) {
    var vertices = [];
    var angle = (Math.PI * 2) / n;

    for (var i = 0; i < n; i++) {
        var vertexX = Math.cos(angle * i) * Width;
        var vertexY = Math.sin(angle * i) * Height;
        vertices.push(new b2Vec2(vertexX, vertexY));
    }

    fixDef.shape = new b2PolygonShape();
    fixDef.shape.SetAsArray(vertices, n);
    bodyDef.position.x = x;
    bodyDef.position.y = y;

    var objBody = world.CreateBody(bodyDef);
    objBody.CreateFixture(fixDef);

    var initialVelocityX = vx; // 범위: -5에서 5까지
    var initialVelocityY = vy; // 범위: -5에서 5까지
    objBody.SetLinearVelocity(new b2Vec2(initialVelocityX, initialVelocityY));

    var initialAngularVelocity = av; // 범위: -5에서 5까지
    objBody.SetAngularVelocity(initialAngularVelocity);

    return objBody;
}

function createWeldJoint(bodyA, bodyB, anchor, b2WeldJointDef) {
    var jointDef = new b2WeldJointDef();
    jointDef.Initialize(bodyA, bodyB, anchor);
    
    return world.CreateJoint(jointDef);
}

function createRevoluteJoint(bodyA, bodyB, anchor, b2RevoluteJointDef) {
    var jointDef = new b2RevoluteJointDef();
    jointDef.Initialize(bodyA, bodyB, anchor);
    
    return world.CreateJoint(jointDef);
}

function update() {
    world.Step(1 / 120, 10, 10);
    
    // Clear the canvas
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    
    // Draw trail
    for (var i = positions.length - 1; i >= 0; i--) {
        var position = positions[i];
        var alpha = (trailOpacity / trailLength) * (i + 1); // 투명도 계산 (역순)
        
        ctx.save();
        ctx.translate(position.x, position.y);
        
        // Draw a circle
        ctx.beginPath();
        ctx.arc(0, 0, 5, 0, 2 * Math.PI);
        ctx.fillStyle = "rgba(255, 255, 255, " + alpha + ")";
        ctx.fill();
        
        ctx.restore();
    }
    
    // Update positions array
    positions.push({ x: end.GetPosition().x, y: end.GetPosition().y });
    if (positions.length > trailLength) {
        positions.shift();
    }
    
    // Draw all bodies
    for (var body = world.GetBodyList(); body; body = body.GetNext()) {
        var fixture = body.GetFixtureList();
        if (fixture) { // Only proceed if the body has a fixture
            var shape = fixture.GetShape();
            var type = shape.GetType();

            ctx.save();
            ctx.translate(body.GetPosition().x, body.GetPosition().y);
            ctx.rotate(body.GetAngle());
            
            if (type == Box2D.Collision.Shapes.b2Shape.e_circleShape) {
                // Draw a circle
                ctx.beginPath();
                ctx.arc(0, 0, shape.GetRadius(), 0, 2 * Math.PI);
                ctx.fillStyle = "#FFFFFF";
                ctx.fill();
            } else if (type == Box2D.Collision.Shapes.b2Shape.e_polygonShape) {
                // Draw a polygon
                var vertices = shape.GetVertices();
                ctx.beginPath();
                ctx.moveTo(vertices[0].x, vertices[0].y);
                for (var i = 1; i < vertices.length; i++) {
                    ctx.lineTo(vertices[i].x, vertices[i].y);
                }
                ctx.closePath();
                ctx.fillStyle = body.GetType() == Box2D.Dynamics.b2Body.b2_staticBody ? "transparent" : "#FFFFFF";
                ctx.fill();
            }
            
            ctx.restore();
        }
    }

    world.ClearForces();
};
