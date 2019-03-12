#include <mitsuba/render/scene.h>
#include <mitsuba/core/statistics.h>
#include <boost/math/distributions/normal.hpp>

MTS_NAMESPACE_BEGIN

static StatsCounter avgPathLength("Path tracer", "Average path length", EAverage);

class LocalRegressionIntegrator : public SamplingIntegrator {
public:
	LocalRegressionIntegrator(const Properties &props) : SamplingIntegrator(props) {
	}

	/// Unserialize
	LocalRegressionIntegrator(Stream *stream, InstanceManager *manager)
		: SamplingIntegrator(stream, manager) {
		m_film_depth = static_cast<Film *>(manager->getInstance(stream));

		m_rrDepth = stream->readInt();
		m_maxDepth = stream->readInt();
		m_strictNormals = stream->readBool();
		m_hideEmitters = stream->readBool();
		m_maxDist = stream->readFloat();
	}

	/// Serialize
	void serialize(Stream *stream, InstanceManager *manager) const {
		SamplingIntegrator::serialize(stream, manager);
		manager->serialize(stream, m_film_depth.get());

		stream->writeInt(m_rrDepth);
		stream->writeInt(m_maxDepth);
		stream->writeBool(m_strictNormals);
		stream->writeBool(m_hideEmitters);
		stream->writeFloat(m_maxDist);
	}

	/// Preprocess
	bool preprocess(const Scene *scene, RenderQueue *queue, const RenderJob *job,
		int sceneResID, int sensorResID, int samplerResID) {

		if (!SamplingIntegrator::preprocess(scene, queue, job, sceneResID, sensorResID, samplerResID))
			return false;
		
		Sampler *sampler = static_cast<Sampler *>(Scheduler::getInstance()->getResource(samplerResID, 0));
		Sensor *sensor = static_cast<Sensor *>(Scheduler::getInstance()->getResource(sensorResID));
		if (sampler->getClass()->getName() != "IndependentSampler")
			Log(EError, "The error-controlling integrator should only be "
			"used in conjunction with the independent sampler");

		/// m_film_depth 초기화
		Vector2i filmSize = sensor->getFilm()->getSize();

		const AABB &sceneAABB = scene->getAABB();

		Point cameraPosition = scene->getSensor()->getWorldTransform()->eval(0)
			.transformAffine(Point(0.0f));
		m_maxDist = -std::numeric_limits<Float>::infinity();

		for (int i = 0; i < 8; ++i) {
			m_maxDist = std::max(m_maxDist,
				(cameraPosition - sceneAABB.getCorner(i)).length());
		}

		return true;
	}

	/// Li에 인자로 Spectrum depth 같은걸 넣어주고 (renderBlock에서 초기화)
	/// 내부에서 depth를 채운 다음에
	/// renderBlock 에서 depthBlock->put(samplePos, [depth])을 하자
	/// renderBlock for 구문 맨 바깥에서 film->put(depthBlock) 라고 하면 될 듯?
	void renderBlock(const Scene *scene, const Sensor *sensor,
		Sampler *sampler, ImageBlock *block, const bool &stop,
		const std::vector< TPoint2<uint8_t> > &points) const {
		
		/*
		depthBlock 초기화 
		*/
		
		ref<ImageBlock> depthBlock = new ImageBlock(Bitmap::ELuminance, block->getSize(), NULL, 1, true);
		

		Float diffScaleFactor = 1.0f /
			std::sqrt((Float)sampler->getSampleCount());

		bool needsApertureSample = sensor->needsApertureSample();
		bool needsTimeSample = sensor->needsTimeSample();

		RadianceQueryRecord rRec(scene, sampler);
		Point2 apertureSample(0.5f);
		Float timeSample = 0.5f;
		RayDifferential sensorRay;

		block->clear();

		uint32_t queryType = RadianceQueryRecord::ESensorRay;

		if (!sensor->getFilm()->hasAlpha()) /* Don't compute an alpha channel if we don't have to */
			queryType &= ~RadianceQueryRecord::EOpacity;

		for (size_t i = 0; i<points.size(); ++i) {
			Point2i offset = Point2i(points[i]) + Vector2i(block->getOffset());
			if (stop)
				break;

			sampler->generate(offset);

			for (size_t j = 0; j<sampler->getSampleCount(); j++) {
				rRec.newQuery(queryType, sensor->getMedium());
				Point2 samplePos(Point2(offset) + Vector2(rRec.nextSample2D()));

				if (needsApertureSample)
					apertureSample = rRec.nextSample2D();
				if (needsTimeSample)
					timeSample = rRec.nextSample1D();

				Spectrum spec = sensor->sampleRayDifferential(
					sensorRay, samplePos, apertureSample, timeSample);

				sensorRay.scaleDifferential(diffScaleFactor);

				spec *= Li(sensorRay, rRec);
				block->put(samplePos, spec, rRec.alpha);
				sampler->advance();
			}
		}

		/*
		feature film update
		*/
		//m_block_depth->put(depthBlock);
	}

	/// How to compute first-bounce radiance
	Spectrum Li(const RayDifferential &r, RadianceQueryRecord &rRec, Float &depth) const {
		/* Some aliases and local variables */
		const Scene *scene = rRec.scene;
		Intersection &its = rRec.its;
		RayDifferential ray(r);
		Spectrum Li(0.0f);
		bool scattered = false;

		/* Perform the first ray intersection (or ignore if the
		intersection has already been provided). */
		rRec.rayIntersect(ray);
		ray.mint = Epsilon;

		depth = 1.0f - rRec.its.t / m_maxDist; /// Depth visualization

		Spectrum throughput(1.0f);
		Float eta = 1.0f;

		while (rRec.depth <= m_maxDepth || m_maxDepth < 0) {
			if (!its.isValid()) {
				/* If no intersection could be found, potentially return
				radiance from a environment luminaire if it exists */
				if ((rRec.type & RadianceQueryRecord::EEmittedRadiance)
					&& (!m_hideEmitters || scattered))
					Li += throughput * scene->evalEnvironment(ray);
				break;
			}

			const BSDF *bsdf = its.getBSDF(ray);

			/* Possibly include emitted radiance if requested */
			if (its.isEmitter() && (rRec.type & RadianceQueryRecord::EEmittedRadiance)
				&& (!m_hideEmitters || scattered))
				Li += throughput * its.Le(-ray.d);

			/* Include radiance from a subsurface scattering model if requested */
			if (its.hasSubsurface() && (rRec.type & RadianceQueryRecord::ESubsurfaceRadiance))
				Li += throughput * its.LoSub(scene, rRec.sampler, -ray.d, rRec.depth);

			if ((rRec.depth >= m_maxDepth && m_maxDepth > 0)
				|| (m_strictNormals && dot(ray.d, its.geoFrame.n)
				* Frame::cosTheta(its.wi) >= 0)) {

				/* Only continue if:
				1. The current path length is below the specifed maximum
				2. If 'strictNormals'=true, when the geometric and shading
				normals classify the incident direction to the same side */
				break;
			}

			/* ==================================================================== */
			/*                     Direct illumination sampling                     */
			/* ==================================================================== */

			/* Estimate the direct illumination if this is requested */
			DirectSamplingRecord dRec(its);

			if (rRec.type & RadianceQueryRecord::EDirectSurfaceRadiance &&
				(bsdf->getType() & BSDF::ESmooth)) {
				Spectrum value = scene->sampleEmitterDirect(dRec, rRec.nextSample2D());
				if (!value.isZero()) {
					const Emitter *emitter = static_cast<const Emitter *>(dRec.object);

					/* Allocate a record for querying the BSDF */
					BSDFSamplingRecord bRec(its, its.toLocal(dRec.d), ERadiance);

					/* Evaluate BSDF * cos(theta) */
					const Spectrum bsdfVal = bsdf->eval(bRec);

					/* Prevent light leaks due to the use of shading normals */
					if (!bsdfVal.isZero() && (!m_strictNormals
						|| dot(its.geoFrame.n, dRec.d) * Frame::cosTheta(bRec.wo) > 0)) {

						/* Calculate prob. of having generated that direction
						using BSDF sampling */
						Float bsdfPdf = (emitter->isOnSurface() && dRec.measure == ESolidAngle)
							? bsdf->pdf(bRec) : 0;

						/* Weight using the power heuristic */
						Float weight = miWeight(dRec.pdf, bsdfPdf);
						Li += throughput * value * bsdfVal * weight;
					}
				}
			}

			/* ==================================================================== */
			/*                            BSDF sampling                             */
			/* ==================================================================== */

			/* Sample BSDF * cos(theta) */
			Float bsdfPdf;
			BSDFSamplingRecord bRec(its, rRec.sampler, ERadiance);
			Spectrum bsdfWeight = bsdf->sample(bRec, bsdfPdf, rRec.nextSample2D());
			if (bsdfWeight.isZero())
				break;

			scattered |= bRec.sampledType != BSDF::ENull;

			/* Prevent light leaks due to the use of shading normals */
			const Vector wo = its.toWorld(bRec.wo);
			Float woDotGeoN = dot(its.geoFrame.n, wo);
			if (m_strictNormals && woDotGeoN * Frame::cosTheta(bRec.wo) <= 0)
				break;

			bool hitEmitter = false;
			Spectrum value;

			/* Trace a ray in this direction */
			ray = Ray(its.p, wo, ray.time);
			if (scene->rayIntersect(ray, its)) {
				/* Intersected something - check if it was a luminaire */
				if (its.isEmitter()) {
					value = its.Le(-ray.d);
					dRec.setQuery(ray, its);
					hitEmitter = true;
				}
			}
			else {
				/* Intersected nothing -- perhaps there is an environment map? */
				const Emitter *env = scene->getEnvironmentEmitter();

				if (env) {
					if (m_hideEmitters && !scattered)
						break;

					value = env->evalEnvironment(ray);
					if (!env->fillDirectSamplingRecord(dRec, ray))
						break;
					hitEmitter = true;
				}
				else {
					break;
				}
			}

			/* Keep track of the throughput and relative
			refractive index along the path */
			throughput *= bsdfWeight;
			eta *= bRec.eta;

			/* If a luminaire was hit, estimate the local illumination and
			weight using the power heuristic */
			if (hitEmitter &&
				(rRec.type & RadianceQueryRecord::EDirectSurfaceRadiance)) {
				/* Compute the prob. of generating that direction using the
				implemented direct illumination sampling technique */
				const Float lumPdf = (!(bRec.sampledType & BSDF::EDelta)) ?
					scene->pdfEmitterDirect(dRec) : 0;
				Li += throughput * value * miWeight(bsdfPdf, lumPdf);
			}

			/* ==================================================================== */
			/*                         Indirect illumination                        */
			/* ==================================================================== */

			/* Set the recursive query type. Stop if no surface was hit by the
			BSDF sample or if indirect illumination was not requested */
			if (!its.isValid() || !(rRec.type & RadianceQueryRecord::EIndirectSurfaceRadiance))
				break;
			rRec.type = RadianceQueryRecord::ERadianceNoEmission;

			if (rRec.depth++ >= m_rrDepth) {
				/* Russian roulette: try to keep path weights equal to one,
				while accounting for the solid angle compression at refractive
				index boundaries. Stop with at least some probability to avoid
				getting stuck (e.g. due to total internal reflection) */

				Float q = std::min(throughput.max() * eta * eta, (Float) 0.95f);
				if (rRec.nextSample1D() >= q)
					break;
				throughput /= q;
			}
		}

		/* Store statistics */
		avgPathLength.incrementBase();
		avgPathLength += rRec.depth;

		return Li;
	}

	inline Float miWeight(Float pdfA, Float pdfB) const {
		pdfA *= pdfA;
		pdfB *= pdfB;
		return pdfA / (pdfA + pdfB);
	}

	/*
	이 아래로는 그냥 기본 함수들
	*/
	Spectrum E(const Scene *scene, const Intersection &its, const Medium *medium,
		Sampler *sampler, int nSamples, bool includeIndirect) const {
		return SamplingIntegrator::E(scene, its, medium,
			sampler, nSamples, includeIndirect);
	}

	void bindUsedResources(ParallelProcess *proc) const {
		SamplingIntegrator::bindUsedResources(proc);
	}

	void wakeup(ConfigurableObject *parent,
		std::map<std::string, SerializableObject *> &params) {
		SamplingIntegrator::wakeup(this, params);
	}

	void cancel() {
		SamplingIntegrator::cancel();
	}

	std::string toString() const {
		std::ostringstream oss;
		oss << "AdaptiveIntegrator[" << endl
			<< "  maxSamples = " << m_maxSampleFactor << "," << endl
			<< "]";
		return oss.str();
	}

	MTS_DECLARE_CLASS();
private:
	ref<Film> m_film_depth;
	ref<ImageBlock> m_block_depth;
	int m_maxDepth;
	int m_rrDepth;
	bool m_strictNormals;
	bool m_hideEmitters;
	Float m_maxDist;
};

MTS_IMPLEMENT_CLASS_S(LocalRegressionIntegrator, false, SamplingIntegrator)
MTS_EXPORT_PLUGIN(LocalRegressionIntegrator, "Weighted local regression integrator");
MTS_NAMESPACE_END