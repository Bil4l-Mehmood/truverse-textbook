import React, { useState } from 'react';
import { updateProfile } from '../../services/authService';
import { useAuthStore } from '../../store/authStore';
import styles from './AuthPages.module.css';

export default function Questionnaire() {
  const token = useAuthStore((state) => state.token);
  const user = useAuthStore((state) => state.user);
  const setUser = useAuthStore((state) => state.setUser);

  // Redirect if not authenticated
  React.useEffect(() => {
    if (!token || !user) {
      window.location.href = '/auth/signin';
    }
  }, [token, user]);

  const [formData, setFormData] = useState({
    ros2_experience: user?.ros2_experience || 'Beginner',
    robotics_knowledge: user?.robotics_knowledge || 'Beginner',
    gpu_model: user?.gpu_model || '',
    gpu_vram: user?.gpu_vram || '',
    operating_system: user?.operating_system || '',
  });

  const [isLoading, setIsLoading] = useState(false);
  const [apiError, setApiError] = useState('');

  const handleChange = (e: React.ChangeEvent<HTMLInputElement | HTMLSelectElement>) => {
    const { name, value } = e.target;
    setFormData((prev) => ({
      ...prev,
      [name]: value,
    }));
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setApiError('');
    setIsLoading(true);

    try {
      if (!token) throw new Error('Not authenticated');

      const updatedUser = await updateProfile(token, {
        ros2_experience: formData.ros2_experience,
        robotics_knowledge: formData.robotics_knowledge,
        gpu_model: formData.gpu_model || undefined,
        gpu_vram: formData.gpu_vram || undefined,
        operating_system: formData.operating_system || undefined,
      });

      setUser(updatedUser);
      window.location.href = '/';
    } catch (error) {
      const message = error instanceof Error ? error.message : 'Failed to save profile';
      setApiError(message);
    } finally {
      setIsLoading(false);
    }
  };

  const handleSkip = () => {
    window.location.href = '/';
  };

  return (
    <div className={styles.authContainer}>
      <div className={styles.authCard}>
        <h1>Background Information</h1>
        <p className={styles.subtitle}>Help us personalize your learning experience</p>

        {apiError && <div className={styles.error}>{apiError}</div>}

        <form onSubmit={handleSubmit}>
          <div className={styles.formGroup}>
            <label htmlFor="ros2_experience">ROS 2 Experience Level</label>
            <select
              id="ros2_experience"
              name="ros2_experience"
              value={formData.ros2_experience}
              onChange={handleChange}
              disabled={isLoading}
            >
              <option value="Beginner">Beginner</option>
              <option value="Intermediate">Intermediate</option>
              <option value="Advanced">Advanced</option>
              <option value="Expert">Expert</option>
            </select>
          </div>

          <div className={styles.formGroup}>
            <label htmlFor="robotics_knowledge">Robotics Knowledge Level</label>
            <select
              id="robotics_knowledge"
              name="robotics_knowledge"
              value={formData.robotics_knowledge}
              onChange={handleChange}
              disabled={isLoading}
            >
              <option value="Beginner">Beginner</option>
              <option value="Intermediate">Intermediate</option>
              <option value="Advanced">Advanced</option>
              <option value="Expert">Expert</option>
            </select>
          </div>

          <div className={styles.formGroup}>
            <label htmlFor="operating_system">Operating System</label>
            <select
              id="operating_system"
              name="operating_system"
              value={formData.operating_system}
              onChange={handleChange}
              disabled={isLoading}
            >
              <option value="">Not specified</option>
              <option value="Linux">Linux</option>
              <option value="Windows">Windows</option>
              <option value="macOS">macOS</option>
            </select>
          </div>

          <div className={styles.formGroup}>
            <label htmlFor="gpu_model">GPU Model (Optional)</label>
            <input
              id="gpu_model"
              type="text"
              name="gpu_model"
              placeholder="e.g., NVIDIA RTX 3090"
              value={formData.gpu_model}
              onChange={handleChange}
              disabled={isLoading}
            />
          </div>

          <div className={styles.formGroup}>
            <label htmlFor="gpu_vram">GPU VRAM (Optional)</label>
            <input
              id="gpu_vram"
              type="text"
              name="gpu_vram"
              placeholder="e.g., 24GB"
              value={formData.gpu_vram}
              onChange={handleChange}
              disabled={isLoading}
            />
          </div>

          <button type="submit" disabled={isLoading} className={styles.submitBtn}>
            {isLoading ? 'Saving...' : 'Continue to Learning'}
          </button>

          <button
            type="button"
            onClick={handleSkip}
            disabled={isLoading}
            style={{
              width: '100%',
              padding: '12px',
              marginTop: '10px',
              background: '#f0f0f0',
              color: '#333',
              border: 'none',
              borderRadius: '8px',
              cursor: 'pointer',
              fontSize: '16px',
            }}
          >
            Skip for Now
          </button>
        </form>
      </div>
    </div>
  );
}
