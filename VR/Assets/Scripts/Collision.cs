using System.Collections;
using System;
using System.Collections.Generic;
using UnityEngine;


public class Collision : MonoBehaviour
{
    public Vector3 start, end;
    SphereCollider s;
    List<Vector3> vertices;
    public List<Vector3> updatedVertices;

    private Rigidbody body;

    static float elastic = .1f;
    float oneOverMass;
    Vector3 angularMomentum;
    Matrix4x4 inertiaTensor;

    static Vector3 origin = new Vector3(0, 0, 0);
    // Start is called before the first frame update

    void Start()
    {
        // SPHERE COLLIDER
        s = GetComponent<SphereCollider>();
        Vector3 radius = new Vector3(s.radius, s.radius, s.radius);
        start = s.transform.position - radius;
        end = s.transform.position + radius;

        body = GetComponent<Rigidbody>();
        oneOverMass = 1f / body.mass;
        inertiaTensor = Matrix4x4.Scale(body.inertiaTensor);
        angularMomentum = inertiaTensor.MultiplyVector(body.angularVelocity);

        // Get the objects vertices 
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        Vector3[] temp = mesh.vertices;
        HashSet<Vector3> tempvertices = new HashSet<Vector3>();
        foreach (Vector3 v in temp)
        {
            tempvertices.Add(v);
        }
        vertices = new List<Vector3>(tempvertices);
        updatedVertices = new List<Vector3>(tempvertices);
        float velocity = -3f;// -
        float a = 0f;
        if (gameObject.name == "1")
        {
            velocity = 2f;
            a = 0f;
        }
        if (gameObject.name != "0")
        {
            body.velocity = new Vector3(velocity, body.velocity.y, body.velocity.z);
            body.angularVelocity = new Vector3(body.angularVelocity.x, a, body.angularVelocity.z);
        }
    }

    // Update is called once per frame
    void Update()
    {

    }

    private void FixedUpdate()
    {
        Vector3 radius = new Vector3(s.radius, s.radius, s.radius);
        start = s.transform.position - radius;
        end = s.transform.position + radius;
        for (int i = 0; i < updatedVertices.Count; i++)
        {
            updatedVertices[i] = transform.TransformPoint(vertices[i]);
            updatedVertices[i] = roundVector(updatedVertices[i]);
        }
    }
    
    public void impulse(Vector3 normalize, Vector3 collisionPoint, GameObject b, Collision c)
    {
        
        Rigidbody body2 = b.GetComponent<Rigidbody>();
        Collider collider1 = body.GetComponent<Collider>();
        Collider collider2 = body2.GetComponent<Collider>();

        Quaternion rotation1 = body.rotation;
        Matrix4x4 rotationMatrix = Matrix4x4.TRS(Vector3.zero, rotation1, Vector3.one);
        Matrix4x4 I1 = Matrix4x4.Scale(body.inertiaTensor);

        Quaternion rotation2 = body2.rotation;
        Matrix4x4 rotationMatrix2 = Matrix4x4.TRS(Vector3.zero, rotation2, Vector3.one);
        Matrix4x4 I2 = Matrix4x4.Scale(body.inertiaTensor);

        Matrix4x4 inertiaTensorInverse = rotationMatrix * I1.inverse * rotationMatrix.transpose;
        Matrix4x4 inertiaTensorInverse2 = rotationMatrix2 * I2.inverse * rotationMatrix2.transpose;

        Vector3 r1 = collisionPoint - body.position;
        Vector3 r2 = collisionPoint - body2.position;

        Vector3 v1 = body.velocity + Vector3.Cross(body.angularVelocity, r1);
        Vector3 v2 = body2.velocity + Vector3.Cross(body2.angularVelocity, r2);


        Vector3 v = v2 - v1;

        float Numerator = (float)Vector3.Dot((-1.0f - elastic) * v, normalize);

        Vector3 ia = inertiaTensorInverse * Vector3.Cross(r1, normalize);
        Vector3 ib = inertiaTensorInverse2 * Vector3.Cross(r2, normalize);

        Vector3 Ia = Vector3.Cross(ia, r1);
        Vector3 Ib = Vector3.Cross(ib, r2);

        float Denominator = (1f / body2.mass) + (1f / body.mass) + Vector3.Dot((Ia + Ib), normalize);

        float j = Numerator / Denominator;

        body.velocity -= (j / body.mass) * normalize;
        body2.velocity += (j / body2.mass) * normalize;
        Matrix4x4 s = m(inertiaTensorInverse, j);
        Matrix4x4 ss = m(inertiaTensorInverse, j);

        body.angularVelocity -= s.MultiplyVector(Vector3.Cross(r1, normalize));
        s = m(inertiaTensorInverse2, j);
        body2.angularVelocity += ss.MultiplyVector(Vector3.Cross(r2, normalize));

        // -------------------------------------------------------------------------------------------------------------------------------------------------------------
        // Rigidbody body2 = b.GetComponent<Rigidbody>();
        // Vector3 R = collisionPoint - body2.position;

        // Vector3 Velocity = body2.velocity + Vector3.Cross(body2.angularVelocity, R);

        // // Compute the inverse of the inertia tensor
        // Matrix4x4 inverseInertiaTensor = c.inertiaTensor.inverse;
        // // Compute the angular momentum

        // float ImpulseNumerator = -(1f + elastic) * Vector3.Dot(Velocity, normalize);

        // float ImpulseDenominator = c.oneOverMass + Vector3.Dot(Vector3.Cross(inverseInertiaTensor * Vector3.Cross(R, normalize), R), normalize);

        // Vector3 Impulse = (float)(ImpulseNumerator / ImpulseDenominator) * normalize;
        // Debug.Log(oneOverMass + "    " + ImpulseNumerator + "   " + ImpulseDenominator);

        // // body2.AddForce((c.oneOverMass * Impulse), ForceMode.VelocityChange);
        // // apply impulse to primary quantities
        // body2.velocity += (c.oneOverMass * Impulse);

        // angularMomentum += Vector3.Cross(R, Impulse);// -----------------
        //                                              // body2.AddTorque((inverseInertiaTensor * angularMomentum) ,ForceMode.VelocityChange);

        // //compute affected auxiliary quantities
        // body2.angularVelocity = inverseInertiaTensor * angularMomentum;

        // ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


        // float i;

        // if(gameObject.name == "0"){// A is solid, B is the one moving
        //     Rigidbody body2 = b.GetComponent<Rigidbody>();
        //     Collider collider2 = body2.GetComponent<Collider>();
        //     Vector3 rB = collisionPoint - body2.centerOfMass; 
        //     Vector3 vAB = (- (body2.velocity - Vector3.Cross(body2.angularVelocity, rB))); // -------------------------------
        //     float v = Vector3.Dot(vAB, normalize);   
        //     //numerator
        //     float e = -1 * (1 + elastic);
        //     float numerator = e * v;

        //     // denominator
        //     float vv = Vector3.Dot(normalize,normalize);
        //     float m2 = (1f/body2.mass);
        //     float m = m2;
        //     float inertia2 = (1f/6) * body2.mass * (collider2.bounds.size.x *collider2.bounds.size.x);

        //     Vector3 rBN = Vector3.Cross(rB, normalize);
        //     rBN = inertia2 * rBN ;
        //     Vector3 RB = Vector3.Cross(rBN, rB);
        //     Vector3 R = RB;
        //     float denominator = m + Vector3.Dot(R, normalize);

        //     i = numerator / denominator;

        //     body2.velocity = body2.velocity - ((i * normalize)/body2.mass);
        //     body2.angularVelocity = body2.angularVelocity - Vector3.Cross(rB, ((i * normalize) * inertia2));
        //     body2.Sleep();
        // }// A is moving and B is solid
        // else if(b.name == "0"){
        //     Collider collider1 = body.GetComponent<Collider>();
        //     Vector3 rA = collisionPoint - body.centerOfMass; // -----------------------
        //     Vector3 vAB = ((body.velocity + Vector3.Cross(body.angularVelocity, rA))); 
        //     float v = Vector3.Dot(vAB, normalize);
        //     //numerator
        //     float e = -1 * (1 + elastic);
        //     float numerator = e * v;

        //     // denominator
        //     float vv = Vector3.Dot(normalize,normalize);
        //     float m1 = (1f/body.mass);
        //     float m = m1 ;
        //     // Debug.Log(k);
        //     float inertia1 = (1f/6) * body.mass * (collider1.bounds.size.x *collider1.bounds.size.x);
        //     Debug.Log(inertia1);
        //     Vector3 rAN = Vector3.Cross(rA, normalize);
        //     rAN = inertia1 * rAN ;
        //     Vector3 RA = Vector3.Cross(rAN, rA);

        //     Vector3 R = RA;
        //     float denominator = m + Vector3.Dot(R, normalize);

        //     i = numerator / denominator;

        //     body.velocity = body.velocity + ((i * normalize)/body.mass);
        //     body.angularVelocity = body.angularVelocity + Vector3.Cross(rA, ((i * normalize) * inertia1));
        //     body.Sleep();
        // }
        // else { // A & B are moving
        //     Rigidbody body2 = b.GetComponent<Rigidbody>();
        //     Collider collider1 = body.GetComponent<Collider>();
        //     Collider collider2 = body2.GetComponent<Collider>();
        //     Vector3 rA = collisionPoint - body.centerOfMass;
        //     Vector3 rB = collisionPoint - body2.centerOfMass; 
        //     Vector3 vAB = ((body.velocity + Vector3.Cross(body.angularVelocity, rA)) - (body2.velocity - Vector3.Cross(body2.angularVelocity, rB)));
        //     float v = (float)Vector3.Dot(vAB, normalize);
        //     //numerator
        //     float e = (float)(-1 * (1 + elastic));
        //     float numerator = (float)(e * v);

        //     // denominator
        //     float vv = (float)Vector3.Dot(normalize,normalize);
        //     float m1 = (1f/body.mass);
        //     float m2 = (1f/body2.mass);
        //     float m = (float)(m1 + m2);
        //     float inertia1 = (1f/6) * body.mass * (collider1.bounds.size.x *collider1.bounds.size.x);
        //     float inertia2 = (1f/6) * body2.mass * (collider2.bounds.size.x *collider2.bounds.size.x);
        //     inertia1 = 0f;
        //     inertia2 = 0f;

        //     Vector3 rAN = Vector3.Cross(rA, normalize);
        //     rAN = inertia1 * rAN ;
        //     Vector3 RA = Vector3.Cross(rAN, rA);
        //     Vector3 rBN = Vector3.Cross(rB, normalize);
        //     rBN = inertia2 * rBN ;
        //     Vector3 RB = Vector3.Cross(rBN, rB);
        //     Vector3 R = RA + RB;
        //     float denominator = (float)(m + (float)(Vector3.Dot(R, normalize)));

        //     i = numerator / denominator;

        //     body.velocity = body.velocity + ((i * normalize)/body.mass);
        //     body2.velocity = body2.velocity - ((i * normalize)/body2.mass); 
        //     body2.Sleep();
        //     body.Sleep();
        //     // body.angularVelocity = body.angularVelocity + Vector3.Cross(rA, ((i * normalize) * inertia1));
        //     // body2.angularVelocity = body2.angularVelocity - Vector3.Cross(rB, ((i * normalize) * inertia2));
        //     }
    }

    public static Matrix4x4 m(Matrix4x4 mat, float a)
    {
        Matrix4x4 result = new Matrix4x4();
        for (int row = 0; row < 4; row++)
        {
            for (int col = 0; col < 4; col++)
            {
                result[row, col] = mat[row, col] * a;
            }
        }
        return result;
    }


    public Vector3 findFurthestPoint(Vector3 direction)
    {
        Vector3 maxPoint = new Vector3();
        float maxDistance = float.MinValue;
        foreach (Vector3 vertex in updatedVertices)
        {
            float distance = Vector3.Dot(vertex, direction);
            distance = round(distance);
            if (distance > maxDistance)
            {
                maxDistance = distance;
                maxPoint = vertex;
            }
        }
        return maxPoint;
    }

    public Vector3 findFurthestPointSphere(Vector3 direction)
    {
        //TODO: add support function for sphere;
        return new Vector3(0, 0, 0);
    }
    private float round(float x)
    {
        return Mathf.Round(x * 1000f) / 1000f;
    }

    private Vector3 roundVector(Vector3 a)
    {
        return new Vector3(round(a.x), round(a.y), round(a.z));
    }

    // public static void ComputeForces(GameObject rb){
    //     Rigidbody body2 = rb.GetComponent<Rigidbody>();
    //     float oneOverMass = 1f/body2.mass;
    //     if(body2.useGravity){
    //         body2.AddForce(Physics.gravity / oneOverMass); 
    //     }
    // }
}