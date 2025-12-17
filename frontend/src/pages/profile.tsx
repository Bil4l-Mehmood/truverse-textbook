/**
 * User profile page
 */

import React from 'react';
import Layout from '@theme/Layout';
import ProfileView from '../components/Auth/ProfileView';

export default function ProfilePage() {
  return (
    <Layout title="Profile" description="Manage your profile">
      <ProfileView />
    </Layout>
  );
}
