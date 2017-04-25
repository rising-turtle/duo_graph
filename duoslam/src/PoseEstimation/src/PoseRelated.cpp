#include <PoseRelated.h>
#include <cmath>

void crossProduct3D(Eigen::Matrix<double,3,1> v0, Eigen::Matrix<double,3,1> v1, Eigen::Matrix<double,3,1> &vOut)
{
	vOut(0)=v0(1)*v1(2)-v0(2)*v1(1);
	vOut(1)=v0(2)*v1(0)-v0(0)*v1(2);
	vOut(2)=v0(0)*v1(1)-v0(1)*v1(0);
}

void rodrigues_so3_exp(Eigen::Matrix<double,3,1> w, double A, double B, Eigen::Matrix<double,3,3> R)
{

	{
		const double wx2 = (double)w(0)*w(0);
		const double wy2 = (double)w(1)*w(1);
		const double wz2 = (double)w(2)*w(2);
		R(0,0) = 1.0 - B*(wy2 + wz2);
		R(1,1) = 1.0 - B*(wx2 + wz2);
		R(2,2) = 1.0 - B*(wx2 + wy2);
	}
	{
		const double a = A*w(2);
		const double b = B*(w(0)*w(1));
		R(0,1) = b - a;
		R(1,0) = b + a;
	}
	{
		const double a = A*w(1);
		const double b = B*(w(0)*w(2));
		R(0,2) = b + a;
		R(2,0) = b - a;
	}
	{
		const double a = A*w(0);
		const double b = B*(w(1)*w(2));
		R(1,2) = b - a;
		R(2,1) = b + a;
	}

}

void exp_HomogeneousMatrix(const Eigen::Matrix<double,6,1>& mu, Eigen::Matrix4f &ResultMatrix, bool pseudo_exponential)
{
	static const double one_6th = 1.0/6.0;
	static const double one_20th = 1.0/20.0;

	Eigen::Matrix<double,3,1> mu_xyz;
	for (int i=0;i<3;i++)
		mu_xyz(i) = mu(i);

	Eigen::Matrix<double,3,1> w;
	for (int i=0;i<3;i++)
		w(i) = mu(i+3);

	double out_pose_m_coords[3];
	Eigen::Matrix<double,3,3> out_pose_m_ROT;

	const double theta_sq = w.squaredNorm(); // w*w;
	const double theta = std::sqrt(theta_sq);
	double A, B;

	Eigen::Matrix<double,3,1> cross;
	crossProduct3D(w, mu_xyz, cross );

	if (theta_sq < 1e-8)
	{
		A = 1.0 - one_6th * theta_sq;
		B = 0.5;

		if (!pseudo_exponential)
		{
			out_pose_m_coords[0] = mu_xyz(0) + 0.5 * cross(0);
			out_pose_m_coords[1] = mu_xyz(1) + 0.5 * cross(1);
			out_pose_m_coords[2] = mu_xyz(2) + 0.5 * cross(2);
		}
	}
	else
	{
		double C;
		if (theta_sq < 1e-6)
		{
			C = one_6th*(1.0 - one_20th * theta_sq);
			A = 1.0 - theta_sq * C;
			B = 0.5 - 0.25 * one_6th * theta_sq;
		}
		else
		{
			const double inv_theta = 1.0/theta;
			A = sin(theta) * inv_theta;
			B = (1 - cos(theta)) * (inv_theta * inv_theta);
			C = (1 - A) * (inv_theta * inv_theta);
		}

		Eigen::Matrix<double,3,1> w_cross;	// = w^cross
		crossProduct3D(w, cross, w_cross );

		if (!pseudo_exponential)
		{
			//result.get_translation() = mu_xyz + B * cross + C * (w ^ cross);
			out_pose_m_coords[0] = mu_xyz(0) + B * cross(0) + C * w_cross(0);
			out_pose_m_coords[1] = mu_xyz(1) + B * cross(1) + C * w_cross(1);
			out_pose_m_coords[2] = mu_xyz(2) + B * cross(2) + C * w_cross(2);
		}
	}

	// 3x3 rotation part:
	rodrigues_so3_exp(w, A, B, out_pose_m_ROT);

	if (pseudo_exponential)
	{
		for (int i=0;i<3;i++)
			out_pose_m_coords[i] = mu_xyz(i);
	}
	// else: has been already filled in above.
	for(int i = 0; i<3; i++)
	{
		for(int j = 0; j< 3; j++)
		{
			ResultMatrix(i,j) = out_pose_m_ROT(i,j);
		}
	}
	for (int i=0;i<3;i++)
		ResultMatrix(i,3)=out_pose_m_coords[i];
	ResultMatrix(3,0)=ResultMatrix(3,1)=ResultMatrix(3,2)=0.;
	ResultMatrix(3,3)=1.;
}
