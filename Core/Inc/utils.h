#define READ_PIN(PORT, PIN) ((PORT->IDR & PIN) ? 1 : 0)
#define SET_PIN(PORT, PIN) PORT->BSRR = PIN
#define RESET_PIN(PORT, PIN) PORT->BSRR = (uint32_t)PIN << 16

void delayMicroseconds(uint32_t microseconds);

template <typename T, typename Total, uint64_t N> class MovingAverage {
public:
    void operator()(T sample) {
        if (m_numSamples < N) {
            m_samples[m_numSamples++] = sample;
            m_total += sample;
        } else {
            T& oldest = m_samples[m_numSamples++ % N];
			m_total += sample - oldest;
            oldest = sample;
        }
    }

    operator T() const {
		if (m_numSamples < N) {
			return m_total / m_numSamples;
		} else {
			return m_total / N;
		}
    }

    void reset() {
        m_numSamples = 0;
        m_total = 0;
    }

private:
    T m_samples[N];
    uint64_t m_numSamples{0};
    Total m_total{0};
};

