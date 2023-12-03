using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class EPACLass
{

    static int EPA_MAX_NUM_ITERATIONS = 100;

    static public (Vector3, Vector3, Vector3, float) EPA(GameObject first, GameObject second, List<Point> simplex)
    {
        Vector3 collisionPoint = new Vector3(0, 0, 0);
        List<Point> polytype = new List<Point>(simplex);
        List<Vector3> faces = new List<Vector3>{
            new Vector3(0, 1, 2),
            new Vector3(0, 3, 1),
            new Vector3(0, 2, 3),
            new Vector3(1, 3, 2)};

        // get the normals and the closest face of the current faces;
        (List<Vector4>, int) temp = getFaceNormals(ref polytype, ref faces);
        List<Vector4> normals = temp.Item1;
        int minFace = temp.Item2;

        Vector3 minNormal = new Vector3();
        Vector3 minFaceVector = new Vector3();
        float minDistance = float.MaxValue;

        // main loop of EPA algorithm
        bool converged = false;
        for (int j = 0; j < EPA_MAX_NUM_ITERATIONS; j++)
        {
            if (minFace >= normals.Count || minFace == -1)
            {
                Debug.Log(j + " " + normals.Count + " " + minFace);
            }
            minNormal = new Vector3(normals[minFace].x, normals[minFace].y, normals[minFace].z);
            minDistance = normals[minFace].w;
            minFaceVector = faces[minFace];
            //get the support point in the direction of closest face's normal
            Point support = new Point();
            getSupport(first, second, minNormal, ref support);
            // Vector3 support = getSupport(first, second, minNormal);
            float supportDistance = Vector3.Dot(minNormal, support.p);
            //Compare between the distance to support point and to the closest face
            if (Math.Abs(supportDistance - minDistance) > 0.001f)
            {
                minDistance = float.MaxValue;
            }
            else
            {
                converged = true;
            }
            // if distance is more than 0, then we continue, because that means we didnt reach a face of the minkowski sum

            // to add the new point we also need to fix the faces so this is the step : 
            List<(int, int)> uniqueEdges = new List<(int, int)>();
            // we get the facess that the support point can "see"
            for (int i = 0; i < normals.Count; i++)
            {
                if (SameDirection(new Vector3(normals[i].x, normals[i].y, normals[i].z), support.p))
                {
                    //the support face can see this face

                    addEdgeIfUnique(ref uniqueEdges, ((int)faces[i].x, (int)faces[i].y));
                    addEdgeIfUnique(ref uniqueEdges, ((int)faces[i].y, (int)faces[i].z));
                    addEdgeIfUnique(ref uniqueEdges, ((int)faces[i].z, (int)faces[i].x));

                    faces[i] = faces[faces.Count - 1];
                    normals[i] = normals[normals.Count - 1];
                    faces.RemoveAt(faces.Count - 1);
                    normals.RemoveAt(normals.Count - 1);
                    i--;
                }
            }


            // add the new faces
            List<Vector3> newFaces = new List<Vector3>();
            for (int i = 0; i < uniqueEdges.Count; i++)
            {
                newFaces.Add(new Vector3(uniqueEdges[i].Item1, uniqueEdges[i].Item2, polytype.Count));
            }
            polytype.Add(support);

            //get the new Normals
            temp = getFaceNormals(ref polytype, ref newFaces);
            List<Vector4> newNormals = temp.Item1;
            int newMinFace = temp.Item2;

            // get the second closest face of the old faces;
            float oldMinDistance = float.MaxValue;
            for (int i = 0; i < normals.Count; i++)
            {
                if (normals[i].w < oldMinDistance)
                {
                    oldMinDistance = normals[i].w;
                    minFace = i;
                }
            }

            //check which is closer the old closest face, or the new closest face
            if (newNormals.Count > 0 && newNormals[newMinFace].w < oldMinDistance)
            {
                minFace = newMinFace + normals.Count;
            }

            //merge old normals and faces, with the new normals and faces 
            normals.AddRange(newNormals);
            faces.AddRange(newFaces);
            if (supportDistance < minDistance) // ********************************************************************
            {
                minDistance = supportDistance;
                collisionPoint = support.p;
            }
            if (converged) break;
        }
        if (!converged)
        {
            Debug.Log("not convereged!");
        }
        //minNormal, minDistance
        //project origin on closest face:
        Vector3 origin = new Vector3(0, 0, 0);
        float distance = Vector3.Dot(origin, minNormal) + minDistance;
        Vector3 projectedOrigin = origin - (minNormal * distance);
        float u = new float(), v = new float(), w = new float(); ;
        Barycentric(polytype[(int)minFaceVector.x].p, polytype[(int)minFaceVector.y].p, polytype[(int)minFaceVector.z].p, origin, ref u, ref v, ref w);
        Vector3 localA = polytype[(int)minFaceVector.x].a * u + polytype[(int)minFaceVector.y].a * v + polytype[(int)minFaceVector.z].a * w;
        Vector3 localB = polytype[(int)minFaceVector.x].b * u + polytype[(int)minFaceVector.y].b * v + polytype[(int)minFaceVector.z].b * w;
        // Debug.Log("collision point: " + collisionPoint + " " + localA);
        return (minNormal, localA, localB, minDistance + 0.001f);
    }

    static private void Barycentric(Vector3 a, Vector3 b, Vector3 c, Vector3 p, ref float u, ref float v, ref float w)
    {
        Vector3 v0 = b - a, v1 = c - a, v2 = p - a;
        float dis_00 = Vector3.Dot(v0, v0);
        float dis_01 = Vector3.Dot(v0, v1);
        float dis_11 = Vector3.Dot(v1, v1);
        float dis_20 = Vector3.Dot(v2, v0);
        float dis_21 = Vector3.Dot(v2, v1);
        float denom = dis_00 * dis_11 - dis_01 * dis_01;
        v = (dis_11 * dis_20 - dis_01 * dis_21) / denom;
        w = (dis_00 * dis_21 - dis_01 * dis_20) / denom;
        u = 1.0f - v - w;
    }
    static private void addEdgeIfUnique(ref List<(int, int)> edges, (int, int) pair)
    {
        bool removed = edges.Remove((pair.Item2, pair.Item1));
        if (removed)
        {
            return;
        }
        else
        {
            edges.Add(pair);
        }

    }
    static private (List<Vector4>, int) getFaceNormals(ref List<Point> polytype, ref List<Vector3> faces)
    {
        List<Vector4> normals = new List<Vector4>();
        int minTriangle = 0;
        float minDistance = float.MaxValue;
        for (int i = 0; i < faces.Count; i++)
        {
            Vector3 a = polytype[(int)faces[i].x].p;
            Vector3 b = polytype[(int)faces[i].y].p;
            Vector3 c = polytype[(int)faces[i].z].p;

            Vector3 normal = Vector3.Normalize(Vector3.Cross((b - a), (c - a)));

            float distance = Vector3.Dot(normal, a);

            if (distance < 0)
            {
                distance *= -1;
                normal *= -1;
            }

            if (distance < minDistance)
            {
                minDistance = distance;
                minTriangle = i;
            }
            normals.Add(new Vector4(normal.x, normal.y, normal.z, distance));
        }

        return (normals, minTriangle);
    }

    private static Vector3 getSupport(GameObject first, GameObject second, Vector3 direction, ref Point point)
    {
        Collision collision1 = first.GetComponent<Collision>();
        Collision collision2 = second.GetComponent<Collision>();
        point.a = collision1.findFurthestPoint(direction);
        point.b = collision2.findFurthestPoint(-direction);
        point.p = point.a - point.b;
        //c = roundVector(c);
        return point.p;
    }


    private static float round(float x)
    {
        return Mathf.Round(x * 1000f) / 1000f;
    }

    private static Vector3 roundVector(Vector3 a)
    {
        return new Vector3(round(a.x), round(a.y), round(a.z));
    }


    private static bool SameDirection(Vector3 a, Vector3 b)
    {
        return Vector3.Dot(a, b) > 0;
    }

}
