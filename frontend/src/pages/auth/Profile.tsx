import React, { useState } from 'react';
import { useNavigate } from 'react-router-dom';
import { getProfile, updateProfile } from '../../services/authService';
import { useAuthStore } from '../../store/authStore';
import styles from './AuthPages.module.css';

export default function Profile() {
  const navigate = useNavigate();
  const token = useAuthStore((state) => state.token);
  const user = useAuthStore((state) => state.user);
  const setUser = useAuthStore((state) => state.setUser);
  const logout = useAuthStore((state) => state.logout);

  React.useEffect(() => {
    if (!token || !user) {
      navigate('/auth/signin');
    }
  }, [token, user, navigate]);

  const [isEditing, setIsEditing] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const [apiError, setApiError] = useState('');

  const [formData, setFormData] = useState({
    ros2_experience: user?.ros2_experience || 'Beginner',
    robotics_knowledge: user?.robotics_knowledge || 'Beginner',
    gpu_model: user?.gpu_model || '',
    gpu_vram: user?.gpu_vram || '',
    operating_system: user?.operating_system || '',
  });

  const handleChange = (e: React.ChangeEvent<HTMLInputElement | HTMLSelectElement>) => {
    const { name, value } = e.target;
    setFormData((prev) => ({
      ...prev,
      [name]: value,
    }));
  };

  const handleSave = async (e: React.FormEvent) => {
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
      setIsEditing(false);
    } catch (error) {
      const message = error instanceof Error ? error.message : 'Failed to update profile';
      setApiError(message);
    } finally {
      setIsLoading(false);
    }
  };

  const handleLogout = () => {
    logout();
    navigate('/');
  };

  if (!user) {
    return null;
  }

  return (
    <div className={styles.authContainer}>
      <div className={styles.authCard}>
        <h1>My Profile</h1>

        {apiError && <div className={styles.error}>{apiError}</div>}

        {!isEditing ? (
          <div style={{ paddingTop: '20px' }}>
            <div style={{ marginBottom: '20px', borderBottom: '1px solid #e0e0e0', paddingBottom: '20px' }}>
              <p style={{ margin: '0 0 8px 0', color: '#666', fontSize: '14px' }}>Email</p>
              <p style={{ margin: '0', fontSize: '16px', fontWeight: '500' }}>{user.email}</p>
            </div>
          </div>
        ) : (
          <div style={{ paddingTop: '20px' }}>
            <form onSubmit={handleSave}>
              <div className={styles.formGroup}>
                <label>ROS 2 Experience Level</label>
                <select
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

              <button type="submit" disabled={isLoading} className={styles.submitBtn}>
                {isLoading ? 'Saving...' : 'Save Changes'}
              </button>
            </form>
          </div>
        )}
      </div>
    </div>
  );
}
