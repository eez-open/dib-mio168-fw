#define READ_PIN(PORT, PIN) ((PORT->IDR & PIN) ? 1 : 0)
#define SET_PIN(PORT, PIN) PORT->BSRR = PIN
#define RESET_PIN(PORT, PIN) PORT->BSRR = (uint32_t)PIN << 16

void delayMicroseconds(uint32_t microseconds);

template <typename T, typename Total, size_t MAX_N> class MovingAverage {
public:
    void operator()(T sample) {
        if (m_numSamples < m_N) {
            m_samples[m_numSamples++] = sample;
            m_total += sample;
        } else {
            T& oldest = m_samples[m_numSamples++ % m_N];
			m_total += sample - oldest;
            oldest = sample;
        }
    }

    operator T() const {
		if (m_numSamples < m_N) {
			return m_total / m_numSamples;
		} else {
			return m_total / m_N;
		}
    }

    void reset() {
        m_numSamples = 0;
        m_total = 0;
    }

    void reset(uint16_t N) {
    	m_N = N;
        m_numSamples = 0;
        m_total = 0;
    }

    uint16_t getN() {
    	return m_N;
    }

private:
    uint16_t m_N{1};
    T m_samples[MAX_N];
    uint64_t m_numSamples{0};
    Total m_total{0};
};

