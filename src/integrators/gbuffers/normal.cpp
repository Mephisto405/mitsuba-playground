#include <mitsuba/render/scene.h>
#include <mitsuba/core/statistics.h>

MTS_NAMESPACE_BEGIN

static StatsCounter avgPathLength("Path tracer", "Average path length", EAverage);

class NormalIntegrator : public SamplingIntegrator {
public:
	NormalIntegrator(const Properties &props)
		: SamplingIntegrator(props) {
		Spectrum defaultColor;
		defaultColor.fromLinearRGB(0.5f, 0.5f, 0.5f); /// for shifting from [-1.0:1.0] to [0.0:1.0]
		m_color = props.getSpectrum("color", defaultColor);
	}

	/// Unserialize from a binary data stream
	NormalIntegrator(Stream *stream, InstanceManager *manager)
		: SamplingIntegrator(stream, manager) {
		m_color = Spectrum(stream);
	}

	void serialize(Stream *stream, InstanceManager *manager) const {
		SamplingIntegrator::serialize(stream, manager);
		m_color.serialize(stream);
	}

	/*
	bool preprocess(const Scene *scene, RenderQueue *queue, const RenderJob *job,
		int sceneResID, int sensorResID, int samplerResID) {
		SamplingIntegrator::preprocess(scene, queue, job, sceneResID,
			sensorResID, samplerResID);

		const AABB &sceneAABB = scene->getAABB();

		Point cameraPosition = scene->getSensor()->getWorldTransform()->eval(0).
			transformAffine(Point(0.0f));
		m_maxDepth = -std::numeric_limits<Float>::infinity();

		for (int i = 0; i<8; ++i)
			m_maxDepth = std::max(m_maxDepth,
			(cameraPosition - sceneAABB.getCorner(i)).length());

		return true;
	}
	*/

	Spectrum Li(const RayDifferential &r, RadianceQueryRecord &rRec) const {
		if (rRec.rayIntersect(r)) {
			RayDifferential ray(r);
			TVector3<Float> n = (dot(ray.d, rRec.its.shFrame.n) < 0.f) ? -n : n;

			const TVector3<Float> wn = normalize(rRec.its.toWorld(n));
			Float arr[3] = { wn.x, wn.y, wn.z };
			return (Spectrum(arr) + Spectrum(1.0f)) * m_color;
		}
		return Spectrum(0.0f);
	}

	MTS_DECLARE_CLASS()
private:
	Spectrum m_color;
};

MTS_IMPLEMENT_CLASS_S(NormalIntegrator, false, SamplingIntegrator)
MTS_EXPORT_PLUGIN(NormalIntegrator, "Normal integrator");
MTS_NAMESPACE_END