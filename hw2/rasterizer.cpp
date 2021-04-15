// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f>& positions)
{
	auto id = get_next_id();
	pos_buf.emplace(id, positions);

	return { id };
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i>& indices)
{
	auto id = get_next_id();
	ind_buf.emplace(id, indices);

	return { id };
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f>& cols)
{
	auto id = get_next_id();
	col_buf.emplace(id, cols);

	return { id };
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
	return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(int x, int y, const Vector3f* _v)
{
	// TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
	/*Vector3f a = _v[0];
	Vector3f b = _v[1];
	Vector3f c = _v[2];

	Vector3f q(x, y, 0);

	Vector3f ab = b - a;
	Vector3f bc = c - b;
	Vector3f ca = a - c;

	Vector3f aq = q - a;
	Vector3f bq = q - b;
	Vector3f cq = q - c;

	Vector3f result1 = ab.cross(aq);
	Vector3f result2 = bc.cross(bq);
	Vector3f result3 = ca.cross(cq);
	return result1.z() > 0 && result2.z() > 0 && result3.z() > 0;*/

	Eigen::Vector2f p(x, y);
	Eigen::Vector2f AB = _v[1].head(2) - _v[0].head(2);
	Eigen::Vector2f BC = _v[2].head(2) - _v[1].head(2);
	Eigen::Vector2f CA = _v[0].head(2) - _v[2].head(2);

	Eigen::Vector2f AP = p - _v[0].head(2);
	Eigen::Vector2f BP = p - _v[1].head(2);
	Eigen::Vector2f CP = p - _v[2].head(2);

	// 判断每个z坐标是否统一
	return AB[0] * AP[1] - AB[1] * AP[0] > 0
		&& BC[0] * BP[1] - BC[1] * BP[0] > 0
		&& CA[0] * CP[1] - CA[1] * CP[0] > 0;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
	float c1 = (x * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * y + v[1].x() * v[2].y() - v[2].x() * v[1].y()) / (v[0].x() * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * v[0].y() + v[1].x() * v[2].y() - v[2].x() * v[1].y());
	float c2 = (x * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * y + v[2].x() * v[0].y() - v[0].x() * v[2].y()) / (v[1].x() * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * v[1].y() + v[2].x() * v[0].y() - v[0].x() * v[2].y());
	float c3 = (x * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * y + v[0].x() * v[1].y() - v[1].x() * v[0].y()) / (v[2].x() * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * v[2].y() + v[0].x() * v[1].y() - v[1].x() * v[0].y());
	return { c1,c2,c3 };
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
	auto& buf = pos_buf[pos_buffer.pos_id];
	auto& ind = ind_buf[ind_buffer.ind_id];
	auto& col = col_buf[col_buffer.col_id];

	float f1 = (50 - 0.1) / 2.0;
	float f2 = (50 + 0.1) / 2.0;

	Eigen::Matrix4f mvp = projection * view * model;
	for (auto& i : ind)
	{
		Triangle t;
		Eigen::Vector4f v[] = {
				mvp * to_vec4(buf[i[0]], 1.0f),
				mvp * to_vec4(buf[i[1]], 1.0f),
				mvp * to_vec4(buf[i[2]], 1.0f)
		};
		//Homogeneous division
		for (auto& vec : v) {
			vec /= vec.w();
		}
		//Viewport transformation
		for (auto& vert : v)
		{
			vert.x() = 0.5 * width * (vert.x() + 1.0);
			vert.y() = 0.5 * height * (vert.y() + 1.0);
			vert.z() = vert.z() * f1 + f2;
		}

		for (int i = 0; i < 3; ++i)
		{
			t.setVertex(i, v[i].head<3>());
			t.setVertex(i, v[i].head<3>());
			t.setVertex(i, v[i].head<3>());
		}

		auto col_x = col[i[0]];
		auto col_y = col[i[1]];
		auto col_z = col[i[2]];

		t.setColor(0, col_x[0], col_x[1], col_x[2]);
		t.setColor(1, col_y[0], col_y[1], col_y[2]);
		t.setColor(2, col_z[0], col_z[1], col_z[2]);

		rasterize_triangle(t);
	}
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
	auto v = t.toVector4();

	// TODO : Find out the bounding box of current triangle.
	// 找出三角形的包围盒(AABB)。

	int xMin = INT32_MAX;
	int xMax = INT32_MIN;
	int yMin = INT32_MAX;
	int yMax = INT32_MIN;

	for (auto& vec : v)
	{
		xMin = std::min((int)vec.x(), xMin);
		xMax = std::max((int)vec.x(), xMax);

		yMin = std::min((int)vec.y(), yMin);
		yMax = std::max((int)vec.y(), yMax);
	}

	bool useMSAA = true;
	if (useMSAA)
	{
		int single = 4;
		float pixelCount = single * single;
		for (int i = xMin; i <= xMax; i++)
		{
			for (int j = yMin; j <= yMax; j++)
			{
				float singleVal = 1 / pixelCount; // 1个像素被拆成几个像素，每个像素一半的长宽
				int insideCount = 0;

				float minDepth = FLT_MAX;

				// 循环4个像素有几个在三角形内
				for (int k = 0; k < single; k++) // x
				{
					for (int l = 0; l < single; l++) // y
					{
						float x = i + k * singleVal + (k + 1) * singleVal;
						float y = j + l * singleVal + (l + 1) * singleVal;

						bool inside = insideTriangle(x, y, t.v);
						if (!inside)
						{
							continue;
						}

						insideCount++;
						std::tuple<float, float, float> result = computeBarycentric2D(x, y, t.v);
						float alpha;
						float beta;
						float gamma;
						std::tie(alpha, beta, gamma) = result;
						float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
						float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
						z_interpolated *= w_reciprocal;
						minDepth = std::min(minDepth, z_interpolated);
					}
				}

				if (insideCount == 0)
					continue;

				float percent = insideCount / pixelCount;

				int index = get_index(i, j);
				float depth = depth_buf[index];
				if (minDepth < depth)
				{
					depth_buf[index] = minDepth;
					Eigen::Vector3f color = t.getColor() * percent;
					Eigen::Vector3f point(i, j, minDepth);
					// TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
					set_pixel(point, color);
				}
			}
		}
	}
	else
	{
		for (int i = xMin; i <= xMax; i++)
		{
			for (int j = yMin; j <= yMax; j++)
			{
				float x = i + 0.5f;
				float y = j + 0.5f;
				bool inside = insideTriangle(x, y, t.v);

				if (!inside)
					continue;

				//如果在内部，则将其位置处的插值深度值(interpolated depth value) 与深度缓冲区(depth buffer) 中的相应值进行比较。
				// If so, use the following code to get the interpolated z value.
				//auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
				//float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
				//float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
				//z_interpolated *= w_reciprocal;

				std::tuple<float, float, float> result = computeBarycentric2D(x, y, t.v);
				float alpha;
				float beta;
				float gamma;
				std::tie(alpha, beta, gamma) = result;
				float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
				float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
				z_interpolated *= w_reciprocal;

				int index = get_index(i, j);
				float depth = depth_buf[index];
				if (z_interpolated < depth)
				{
					depth_buf[index] = z_interpolated;
					Eigen::Vector3f color = t.getColor();
					Eigen::Vector3f point(i, j, z_interpolated);
					// TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
					set_pixel(point, color);
				}
			}
		}
	}
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
	model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
	view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
	projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
	if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
	{
		std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{ 0, 0, 0 });
	}
	if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
	{
		std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
	}
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
	frame_buf.resize(w * h);
	depth_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
	return (height - 1 - y) * width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
	//old index: auto ind = point.y() + point.x() * width;
	auto ind = (height - 1 - point.y()) * width + point.x();
	frame_buf[ind] = color;

}

// clang-format on