#include <mitsuba/render/scene.h>
#include <mitsuba/core/statistics.h>

MTS_NAMESPACE_BEGIN

static StatsCounter avgPathLength("Path tracer", "Average path length", EAverage);

class DepthIntegrator : public SamplingIntegrator {
public:
	DepthIntegrator(const Properties &props)
		: SamplingIntegrator(props) { 
		Spectrum defaultColor;
		defaultColor.fromLinearRGB(1.0f, 1.0f, 1.0f); /// black
		m_color = props.getSpectrum("color", defaultColor);
	}

	/// Unserialize from a binary data stream
	DepthIntegrator(Stream *stream, InstanceManager *manager)
		: SamplingIntegrator(stream, manager) { 
		m_color = Spectrum(stream);
		m_maxDepth = stream->readFloat();
	}

	void serialize(Stream *stream, InstanceManager *manager) const {
		SamplingIntegrator::serialize(stream, manager);
		m_color.serialize(stream);
		stream->writeFloat(m_maxDepth);
	}

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

	Spectrum Li(const RayDifferential &r, RadianceQueryRecord &rRec) const {
		if (rRec.rayIntersect(r)) {
			Float distance = rRec.its.t;
			return Spectrum(1.0f - distance / m_maxDepth) * m_color;
		}
		return Spectrum(0.0f);
	}

	MTS_DECLARE_CLASS()
private:
	Spectrum m_color;
	Float m_maxDepth;
};

MTS_IMPLEMENT_CLASS_S(DepthIntegrator, false, SamplingIntegrator)
MTS_EXPORT_PLUGIN(DepthIntegrator, "Depth integrator");
MTS_NAMESPACE_END