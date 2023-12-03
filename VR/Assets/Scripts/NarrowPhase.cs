using System.Collections;
using System.Collections.Generic;
using UnityEngine;
public struct Point
{
    public Vector3 p;
    public Vector3 a;
    public Vector3 b;
    public Point(Vector3 pp, Vector3 aa, Vector3 bb)
    {
        p = pp;
        a = aa;
        b = bb;
    }

}

public class NarrowPhaseClass
{
    static public void NarrowPhase(ref HashSet<(string, string)> intersectedObjects)
    {
        List<(string, string, List<Point>)> collidedObjects = new List<(string, string, List<Point>)>();
        foreach ((string, string) intersectedPair in intersectedObjects)
        {
            Collision collision = GameObject.Find(intersectedPair.Item1).GetComponent<Collision>();

            (bool, List<Point>) gjkOutput = GJKClass.GJK(GameObject.Find(intersectedPair.Item1), GameObject.Find(intersectedPair.Item2));
            // Debug.Log(intersectedPair.Item1 + " and" + intersectedPair.Item2 + "are colliding: " + gjkOutput.Item1);
            if (gjkOutput.Item1)
            {
                // Debug.Log("yes");
                collidedObjects.Add((intersectedPair.Item1, intersectedPair.Item2, gjkOutput.Item2));
            }
        }
        // EPA get normals and penetration depth of the two collided objects
        foreach ((string, string, List<Point>) collidedPair in collidedObjects)
        {
            Collision collision = GameObject.Find(collidedPair.Item1).GetComponent<Collision>();
            (Vector3, Vector3, Vector3, float) epaOutput = EPACLass.EPA(GameObject.Find(collidedPair.Item1), GameObject.Find(collidedPair.Item2), collidedPair.Item3);
            //  return (minNormal, localA, localB, minDistance + 0.001f);
            // Debug.Log(collidedPair.Item1 + " and" + collidedPair.Item2 +
            //   "are colliding, and penetration distance and vector are: " + epaOutput.Item4+" "+epaOutput.Item1 + "and the point for a  is: " + epaOutput.Item2+ "and for b: "+ epaOutput.Item3);

            Vector3 eee = -1 * epaOutput.Item1;
            GameObject obj1 = GameObject.Find(collidedPair.Item1);
            GameObject obj2 = GameObject.Find(collidedPair.Item2);
            if (obj1.name == "0")
            {
                Collision c2 = GameObject.Find(collidedPair.Item2).GetComponent<Collision>();
                c2.impulse(eee, epaOutput.Item2, obj2, c2);
            }
            else if (obj2.name == "0")
            {
                Collision c1 = GameObject.Find(collidedPair.Item1).GetComponent<Collision>();
                c1.impulse(epaOutput.Item1, epaOutput.Item2, obj1, c1);
            }
            else
            {
                Collision c1 = GameObject.Find(collidedPair.Item1).GetComponent<Collision>();
                Collision c2 = GameObject.Find(collidedPair.Item1).GetComponent<Collision>();
                c1.impulse(eee, epaOutput.Item2, obj1, c1);
                c2.impulse(epaOutput.Item1, epaOutput.Item2, obj2, c2);

            }


        }

    }
}
