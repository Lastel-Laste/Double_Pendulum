// 2D.js

var     world
    ,   positions           = [] // 과거 위치 배열
    ,   trailLength         = 1000 // 잔상 길이
    ,   trailOpacity        = 0.2 // 잔상 투명도

var     b2Vec2              = Box2D.Common.Math.b2Vec2
    ,	b2BodyDef           = Box2D.Dynamics.b2BodyDef
    ,	b2Body              = Box2D.Dynamics.b2Body
    ,	b2FixtureDef        = Box2D.Dynamics.b2FixtureDef
    ,	b2Fixture           = Box2D.Dynamics.b2Fixture
    ,	b2World             = Box2D.Dynamics.b2World
    ,	b2MassData          = Box2D.Collision.Shapes.b2MassData
    ,	b2PolygonShape      = Box2D.Collision.Shapes.b2PolygonShape
    ,	b2CircleShape       = Box2D.Collision.Shapes.b2CircleShape
    ,   b2RevoluteJointDef  = Box2D.Dynamics.Joints.b2RevoluteJointDef
    ,   b2WeldJointDef      = Box2D.Dynamics.Joints.b2WeldJointDef
    ,	b2DebugDraw         = Box2D.Dynamics.b2DebugDraw
    ,   fixDef              = new b2FixtureDef
    ,   bodyDef             = new b2BodyDef


function init() {
    
    world = new b2World(
        new b2Vec2(0, 9.8)    // gravity
    ,   true                  // allow sleep
    )
    
    var groundWidth             = window.innerWidth
    ,   groundHeight            = window.innerHeight
        fixDef.density          = 1.0
        fixDef.friction         = 0.0
        fixDef.restitution      = 0.0
        fixDef.filter.maskBits  = 0
    
    // StaticBody
    bodyDef.type         = b2Body.b2_staticBody

        root             = createCircle(groundWidth / 2, groundHeight / 2, 5, 0, 0, 0)

    // DynamicBody
    bodyDef.type         = b2Body.b2_dynamicBody
    
        edge             = createPolygon(groundWidth / 2 + 100, groundHeight / 2, 4, 100, 3, 0, 0, 0)
    ,   edgeLength       = 100 // The length of the edge
    ,   edgeAngle        = edge.GetAngle() // The current angle of the edge
    ,   otherEndX        = edge.GetPosition().x + edgeLength * Math.cos(edgeAngle)
    ,   otherEndY        = edge.GetPosition().y + edgeLength * Math.sin(edgeAngle)
    
        edge2            = createPolygon(otherEndX+50, otherEndY, 4, 50, 2, 0, 0, 0)
    ,   edge2Length      = 50 // The length of the edge
    ,   edge2Angle       = edge2.GetAngle() // The current angle of the edge
    ,   otherEndX2       = edge2.GetPosition().x + edge2Length * Math.cos(edge2Angle)
    ,   otherEndY2       = edge2.GetPosition().y + edge2Length * Math.sin(edge2Angle)

        edge3            = createPolygon(otherEndX2+25, otherEndY2, 4, 25, 1, 0, 0, 0)
    ,   edge3Length      = 25 // The length of the edge
    ,   edge3Angle       = edge3.GetAngle() // The current angle of the edge
    ,   otherEndX3       = edge3.GetPosition().x + edge3Length * Math.cos(edge3Angle)
    ,   otherEndY3       = edge3.GetPosition().y + edge3Length * Math.sin(edge3Angle)

        end              = createCircle(otherEndX3, otherEndY3, 5, 0, 0, 0)
    
        jointAnchorRoot  = root.GetWorldCenter()
    ,   jointAnchorEdge  = new b2Vec2(otherEndX, otherEndY)
    ,   jointAnchorEdge2 = new b2Vec2(otherEndX2, otherEndY2)
    ,   jointAnchorEdge3 = new b2Vec2(otherEndX3, otherEndY3)

    createRevoluteJoint(root, edge, jointAnchorRoot)
    createRevoluteJoint(edge, edge2, jointAnchorEdge)
    createRevoluteJoint(edge2, edge3, jointAnchorEdge2)
    createWeldJoint(edge3, end, jointAnchorEdge3)

    window.setInterval(update, 1000 / 240)
}

function createCircle(x, y, size, vx, vy, av) {
    fixDef.shape = new b2CircleShape (size)
    bodyDef.position.x              = x
    bodyDef.position.y              = y
    var     initialVelocityX        = vx
    ,       initialVelocityY        = vy
    ,       initialAngularVelocity  = av
    var     objBody                 = world.CreateBody(bodyDef)
            objBody.CreateFixture(fixDef)
            objBody.SetLinearVelocity(new b2Vec2(initialVelocityX, initialVelocityY))
            objBody.SetAngularVelocity(initialAngularVelocity)
    return  objBody
}

function createPolygon(x, y, n, Width, Height, vx, vy, av) {
    var vertices = []
    ,   angle = (Math.PI * 2) / n
    for (let i = 0; i < n; i++) {
        var vertexX = Math.cos(angle * i) * Width
        ,   vertexY = Math.sin(angle * i) * Height
        vertices.push(new b2Vec2(vertexX, vertexY))
    }
    fixDef.shape = new b2PolygonShape()
    fixDef.shape.SetAsArray(vertices, n)
    bodyDef.position.x              = x
    bodyDef.position.y              = y
    var     initialVelocityX        = vx
    ,       initialVelocityY        = vy
    ,       initialAngularVelocity  = av
    var     objBody                 = world.CreateBody(bodyDef)
            objBody.CreateFixture(fixDef)
            objBody.SetLinearVelocity(new b2Vec2(initialVelocityX, initialVelocityY))
            objBody.SetAngularVelocity(initialAngularVelocity)
    return  objBody
}

function createWeldJoint(bodyA, bodyB, anchor) {
    var     jointDef = new b2WeldJointDef()
            jointDef.Initialize(bodyA, bodyB, anchor)
    return  world.CreateJoint(jointDef)
}

function createRevoluteJoint(bodyA, bodyB, anchor) {
    var     jointDef = new b2RevoluteJointDef()
            jointDef.Initialize(bodyA, bodyB, anchor)
    return  world.CreateJoint(jointDef)
}

function update() {
    world.Step(1 / 120, 10, 10)
    
    // Clear the canvas
    ctx.clearRect(0, 0, canvas.width, canvas.height)
    
    // Draw trail
    for (var i = positions.length - 1; i >= 0; i--) {
        var position = positions[i]
        ,   alpha = (trailOpacity / trailLength) * (i + 1) // 투명도 계산 (역순)
        
        ctx.save()
        ctx.translate(position.x, position.y)
        
        // Draw a circle
        ctx.beginPath()
        ctx.arc(0, 0, 5, 0, 2 * Math.PI)
        ctx.fillStyle = "rgba(255, 255, 255, " + alpha + ")"
        ctx.fill()
        
        ctx.restore()
    }
    
    // Update positions array
    positions.push({ x: end.GetPosition().x, y: end.GetPosition().y })
    if (positions.length > trailLength) {
        positions.shift()
    }
    
    // Draw all bodies
    for (var body = world.GetBodyList(); body; body = body.GetNext()) {
        var fixture = body.GetFixtureList()
        if (fixture) { // Only proceed if the body has a fixture
            var shape = fixture.GetShape()
            ,   type = shape.GetType()

            ctx.save()
            ctx.translate(body.GetPosition().x, body.GetPosition().y)
            ctx.rotate(body.GetAngle())
            
            if (type == Box2D.Collision.Shapes.b2Shape.e_circleShape) {
                // Draw a circle
                ctx.beginPath()
                ctx.arc(0, 0, shape.GetRadius(), 0, 2 * Math.PI)
                ctx.fillStyle = "#FFFFFF"
                ctx.fill()
            } else if (type == Box2D.Collision.Shapes.b2Shape.e_polygonShape) {
                // Draw a polygon
                var vertices = shape.GetVertices()
                ctx.beginPath()
                ctx.moveTo(vertices[0].x, vertices[0].y)
                for (var i = 1; i < vertices.length; i++) {
                    ctx.lineTo(vertices[i].x, vertices[i].y)
                }
                ctx.closePath()
                ctx.fillStyle = body.GetType() == Box2D.Dynamics.b2Body.b2_staticBody ? "transparent" : "#FFFFFF"
                ctx.fill()
            }
            
            ctx.restore()
        }
    }

    world.ClearForces()
}