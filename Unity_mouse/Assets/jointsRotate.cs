using UnityEngine;
using System.Collections;

public class jointsRotate : MonoBehaviour {

	GameObject joint1, joint2, joint3, joint4, joint5, joint6, endEff;
	//public Transform target_point;
	public Transform target_noap;

	//Parameter.
	float a1 = 10;    //100mm;
	float a2 = 25;    //250mm;
	float a3 = 13;    //130mm;
	float d1 = 35;    //350mm;
	float d4 = 25;    //250mm;
	float d6 = 17.9f; //85mm + end-effector(94mm);  =>學姊設95.5mm!!!


	public float[] theta_tar = new float[] { 0, 90, 0, 0, 0, 0 };	//target theta
	float[] theta_now = new float[] { 0, 90, 0, 0, 0, 0 };			//present theta(now)
	float[] dtheta = new float[] { 0, 0, 0, 0, 0, 0 };				//delta theta(diff)

	void Awake()  
	{        
		joint1 = GameObject.Find("Joint1");
		joint2 = GameObject.Find("Joint2");
		joint3 = GameObject.Find("Joint3");
		joint4 = GameObject.Find("Joint4");
		joint5 = GameObject.Find("Joint5");
		joint6 = GameObject.Find("Joint6");
		endEff = GameObject.Find("end_effector");
	}

	// Use this for initialization
	void Start () {

	}

	// Update is called once per frame
	void Update () {

		//theta_tar[1] = 30;
		//j123_IK(-target_point.position.z, target_point.position.x, target_point.position.y);	// origianl(x,y,z) => unity(-z,x,y)

		j123456_IK (target_noap);

		//		for (int i = 0; i < 6; i++)
		//			Debug.Log ("theta_tar[" + i + "]= " + theta_tar[i] + "  //  theta_now[" + i + "]= " + theta_now[i]);

		for (int i = 0; i < 6; i++)  ///5?
			dtheta [i] = theta_tar [i] - theta_now [i];

		joint1.transform.RotateAround (Vector3.zero, Vector3.up, dtheta [0]);
		joint2.transform.Rotate (Vector3.up, dtheta [1]);
		joint3.transform.Rotate (Vector3.up, dtheta [2]);
		joint4.transform.Rotate (Vector3.up, dtheta [3]);
		joint5.transform.Rotate (Vector3.up, dtheta [4]);
		joint6.transform.Rotate (Vector3.up, dtheta [5]);


		for (int i = 0; i < 6; i++)		//refresh: after rotate  ///5?
			theta_now [i] = theta_tar [i];

		/*
		Debug.Log ("j1 - "+transform.TransformPoint(joint1.transform.position));
		Debug.Log ("j2 - "+transform.TransformPoint(joint2.transform.position));
		Debug.Log ("j3 - "+transform.TransformPoint(joint3.transform.position));
		Debug.Log ("j4 - "+transform.TransformPoint(joint3.transform.position));
		*/

	}

	void j123_IK(Transform tar)
	{
		/*float pwx = -tar.position.z;
		float pwy = tar.position.x;
		float pwz = tar.position.y;*/

		//****************************************** tar_Eff

		float ax6 = Vector3.Dot(endEff.transform.up, Vector3.back);		// ax = (endEff_z = y in unity) dot (world_x = -z in unity)
		float ay6 = Vector3.Dot (endEff.transform.up, Vector3.right);	// ay = (endEff_z = y in unity) dot (world_y =  x in unity)
		float az6 = Vector3.Dot(endEff.transform.up, Vector3.up);		// az = (endEff_z = y in unity) dot (world_z =  y in unity)

		float pwx, pwy, pwz;
		float px = -tar.position.z;
		float py = tar.position.x;
		float pz = tar.position.y;

		pwx = px - d6 * ax6;
		pwy = py - d6 * ay6;
		pwz = pz - d6 * az6;
		//******************************************

		//theta1. -1
		theta_tar [0] = Mathf.Atan2 (pwy, pwx) ;
		//theta_tar [0] = Mathf.Atan2 (pwy, pwx) + 180 * Mathf.Deg2Rad;
		//Debug.Log ("j1 - "+theta_tar [0]);

		float p1wx, p1wy, p1wz;
		p1wx = Mathf.Cos (theta_tar [0]) * pwx + Mathf.Sin (theta_tar [0]) * pwy - a1;
		p1wy = -pwz + d1;
		p1wz = -Mathf.Sin (theta_tar [0]) * pwx + Mathf.Cos (theta_tar [0]) * pwy;

		float phi = Mathf.Atan2 (p1wy, p1wx);
		float r = Mathf.Sqrt (p1wx * p1wx + p1wy * p1wy);
		float l = Mathf.Sqrt (a3 * a3 + d4 * d4);
		float gamma = Mathf.Atan2 (d4, a3);

		float alpha, beta;
		//theta2. & 3. -1
		if (r > l + a2) {
			theta_tar [1] = phi;
			theta_tar [2] = -gamma;
		} else {
			alpha = Mathf.Acos ((r * r + a2 * a2 - l * l) / (2 * r * a2));
			beta = Mathf.Acos ((r * r - a2 * a2 + l * l) / (2 * r * l));
			theta_tar [1] = phi - alpha;
			theta_tar [2] = alpha + beta - gamma;
		}

		//!!! unity y-axis => -y  &&  Rad => Deg  
		for (int i = 0; i < 6; i++)
			theta_tar [i] = -theta_tar [i] * Mathf.Rad2Deg;
	}

	void j123456_IK(Transform tar)
	{
		float px = -tar.position.z;
		float py = tar.position.x;
		float pz = tar.position.y;

		float nx = Vector3.Dot (-tar.forward, Vector3.back);	// nx = (endEff_x = -z in unity) dot (world_x = -z in unity)
		float ny = Vector3.Dot (-tar.forward, Vector3.right);	// ny = (endEff_x = -z in unity) dot (world_y =  x in unity)
		float nz = Vector3.Dot (-tar.forward, Vector3.up);		// nz = (endEff_x = -z in unity) dot (world_z =  y in unity)
		float ox = Vector3.Dot (tar.right, Vector3.back);		// ox = (endEff_y = x in unity) dot (world_x = -z in unity)
		float oy = Vector3.Dot (tar.right, Vector3.right);		// oy = (endEff_y = x in unity) dot (world_y =  x in unity)
		float oz = Vector3.Dot (tar.right, Vector3.up);			// oz = (endEff_y = x in unity) dot (world_z =  y in unity)
		float ax = Vector3.Dot (tar.up, Vector3.back);			// ax = (endEff_z = y in unity) dot (world_x = -z in unity)
		float ay = Vector3.Dot (tar.up, Vector3.right);			// ay = (endEff_z = y in unity) dot (world_y =  x in unity)
		float az = Vector3.Dot (tar.up, Vector3.up);			// az = (endEff_z = y in unity) dot (world_z =  y in unity)

		/*Debug.Log ("nx "+nx.ToString () + "ny " + ny.ToString () + "nz " + nz.ToString ()+ "\n");
		Debug.Log ("ox "+ox.ToString () + "oy " + oy.ToString () + "oz " + oz.ToString ()+ "\n");
		Debug.Log ("ax "+ax.ToString () + "ay " + ay.ToString () + "az " + az.ToString ()+ "\n");

		Debug.Log ("ax6 "+ax6.ToString () + "ay6 " + ay6.ToString () + "az6 " + az6.ToString ()+ "\n");*/

		Debug.Log ("eff "+endEff.transform.up.ToString () +"\n");
		Debug.Log ("tar "+endEff.transform.up.ToString () +"\n");

		float pwx, pwy, pwz;
		pwx = px - d6 * ax;
		pwy = py - d6 * ay;
		pwz = pz - d6 * az;

		//***theta1. -1
		theta_tar [0] = Mathf.Atan2 (pwy, pwx) ;
		//theta_tar [0] = Mathf.Atan2 (pwy, pwx) + 180 * Mathf.Deg2Rad;
		//Debug.Log ("j1 - "+theta_tar [0]);

		float p1wx, p1wy, p1wz;
		p1wx = Mathf.Cos (theta_tar [0]) * pwx + Mathf.Sin (theta_tar [0]) * pwy - a1;
		p1wy = -pwz + d1;
		p1wz = -Mathf.Sin (theta_tar [0]) * pwx + Mathf.Cos (theta_tar [0]) * pwy;

		float phi = Mathf.Atan2 (p1wy, p1wx);
		float r = Mathf.Sqrt (p1wx * p1wx + p1wy * p1wy);
		float l = Mathf.Sqrt (a3 * a3 + d4 * d4);
		float gamma = Mathf.Atan2 (d4, a3);

		float alpha, beta;
		//***theta2. & 3. -1
		if (r > l + a2) {
			theta_tar [1] = phi;
			theta_tar [2] = -gamma;
		} else {
			alpha = Mathf.Acos ((r * r + a2 * a2 - l * l) / (2 * r * a2));
			beta = Mathf.Acos ((r * r - a2 * a2 + l * l) / (2 * r * l));
			theta_tar [1] = phi - alpha ;
			theta_tar [2] = alpha + beta - gamma;
		}

		//***theta4. & 5. & 6. -1
		float n03x, n03y, n03z, o03x, o03y, o03z, a03x, a03y, a03z;
		float n3x, n3y, n3z, o3x, o3y, o3z, a3x, a3y, a3z;

		n03x = Mathf.Cos (theta_tar [0]) * Mathf.Cos (theta_tar [1] + theta_tar [2]);
		n03y = Mathf.Sin (theta_tar [0]) * Mathf.Cos (theta_tar [1] + theta_tar [2]);
		n03z = -Mathf.Sin (theta_tar [1] + theta_tar [2]);
		o03x = Mathf.Sin (theta_tar [0]);
		o03y = -Mathf.Cos (theta_tar [0]);
		o03z = 0;
		a03x = -Mathf.Cos (theta_tar [0]) * Mathf.Sin (theta_tar [1] + theta_tar [2]);
		a03y = -Mathf.Sin (theta_tar [0]) * Mathf.Sin (theta_tar [1] + theta_tar [2]);
		a03z = -Mathf.Cos (theta_tar [1] + theta_tar [2]);

		//n3x = n03x * nx + n03y * ny + n03z * nz;
		//n3y = o03x * nx + o03y * ny + o03z * nz;
		n3z = a03x * nx + a03y * ny + a03z * nz;
		//o3x = n03x * ox + n03y * oy + n03z * oz;
		//o3y = o03x * ox + o03y * oy + o03z * oz;
		o3z = a03x * ox + a03y * oy + a03z * oz;
		a3x = n03x * ax + n03y * ay + n03z * az;
		a3y = o03x * ax + o03y * ay + o03z * az;
		a3z = a03x * ax + a03y * ay + a03z * az;

		// theta456 -1
		theta_tar [3] = Mathf.Atan2 (-a3y, -a3x) ;
		theta_tar [4] = Mathf.Atan2 (Mathf.Sqrt (a3x * a3x + a3y * a3y), a3z);
		theta_tar [5] = Mathf.Atan2 (-o3z, n3z) ;

		if (theta_tar [4] < 0) {
			// theta456 -2
			theta_tar [3] = Mathf.Atan2 (a3y, a3x) ;
			theta_tar [4] = Mathf.Atan2 (-Mathf.Sqrt (a3x * a3x + a3y * a3y), a3z) - 90 * Mathf.Deg2Rad;
			theta_tar [5] = Mathf.Atan2 (o3z, -n3z);
		}

		//!!! unity y-axis => -y  &&  Rad => Deg  
		for (int i = 0; i < 6; i++)
			theta_tar [i] = -theta_tar [i] * Mathf.Rad2Deg;
	}
}