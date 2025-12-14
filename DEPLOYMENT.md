# Deployment Guide

This guide explains how to deploy the **Physical AI & Humanoid Robotics Textbook** to Vercel.

## Prerequisites

- Git repository initialized and committed
- GitHub account
- Vercel account (free tier is sufficient)

## Quick Deploy to Vercel

### Option 1: Deploy via Vercel Dashboard (Recommended)

1. **Push to GitHub**:
   ```bash
   git add .
   git commit -m "Initial Docusaurus setup with course content"
   git push origin main
   ```

2. **Import to Vercel**:
   - Visit [vercel.com/new](https://vercel.com/new)
   - Click **"Import Project"**
   - Select your GitHub repository (`Truverse` or `ai-textbook-platform`)
   - Vercel will auto-detect Docusaurus configuration

3. **Configure Build Settings**:
   - **Framework Preset**: Docusaurus
   - **Root Directory**: `frontend`
   - **Build Command**: `npm run build` (auto-detected)
   - **Output Directory**: `build` (auto-detected)
   - **Install Command**: `npm install` (auto-detected)

4. **Environment Variables** (if needed later):
   - `REACT_APP_API_BASE_URL`: Your backend Railway URL (for RAG chatbot integration)
   - Add this later when backend is deployed

5. **Deploy**:
   - Click **"Deploy"**
   - Vercel will build and deploy in ~2-3 minutes
   - Your site will be live at `https://<your-project>.vercel.app`

### Option 2: Deploy via Vercel CLI

1. **Install Vercel CLI**:
   ```bash
   npm install -g vercel
   ```

2. **Login to Vercel**:
   ```bash
   vercel login
   ```

3. **Deploy from Frontend Directory**:
   ```bash
   cd frontend
   vercel
   ```

4. **Follow Prompts**:
   - **Set up and deploy?** ’ Yes
   - **Which scope?** ’ Your username/team
   - **Link to existing project?** ’ No
   - **Project name?** ’ `truverse-textbook` (or custom name)
   - **In which directory is your code located?** ’ `./` (already in frontend/)

5. **Production Deployment**:
   ```bash
   vercel --prod
   ```

## Verify Deployment

After deployment, test your site:

1. **Visit Deployed URL**: `https://<your-project>.vercel.app`

2. **Check Key Pages**:
   - [x] Homepage loads with Physical AI branding
   - [x] Sidebar navigation works (all modules visible)
   - [x] [Introduction](/docs/intro) page renders correctly
   - [x] [Quarter Overview](/docs/quarter-overview) displays full curriculum
   - [x] [Hardware Requirements](/docs/hardware-requirements) shows specifications
   - [x] [Weeks 1-2](/docs/weeks-01-02/) module content loads
   - [x] [Weeks 3-4](/docs/weeks-03-04/) module content loads

3. **Test Responsive Design**:
   - Desktop (1920x1080)
   - Tablet (768x1024)
   - Mobile (375x667)

4. **Run Lighthouse Audit**:
   - Open Chrome DevTools ’ Lighthouse
   - Run audit (Performance, Accessibility, SEO)
   - Target scores: >90 for all categories

## Custom Domain (Optional)

To use a custom domain like `textbook.yourdomain.com`:

1. **Add Domain in Vercel**:
   - Go to project **Settings** ’ **Domains**
   - Add your custom domain
   - Follow DNS configuration instructions

2. **DNS Configuration**:
   - Add CNAME record: `textbook` ’ `cname.vercel-dns.com`
   - Or A record for root domain: `76.76.21.21`

3. **SSL Certificate**:
   - Vercel automatically provisions SSL via Let's Encrypt
   - HTTPS will be enabled within minutes

## Continuous Deployment

Vercel automatically redeploys when you push to GitHub:

```bash
# Make changes to content
nano frontend/docs/intro.md

# Commit and push
git add .
git commit -m "Update introduction page"
git push origin main

# Vercel auto-deploys in ~2 minutes
```

**Check Deployment Status**:
- Visit [vercel.com/dashboard](https://vercel.com/dashboard)
- View deployment logs and build output

## Troubleshooting

### Build Fails with "MDX compilation error"

**Cause**: Special characters (`<`, `>`, `{`, `}`) in Markdown are interpreted as JSX.

**Solution**: Escape them or use different notation:
```markdown
<!-- BAD -->
**F (<60%)**: Failing grade

<!-- GOOD -->
**F (below 60%)**: Failing grade
```

### Site Loads but Sidebar is Empty

**Cause**: Sidebar configuration issue or missing files.

**Solution**: Check `frontend/sidebars.ts` and ensure all referenced files exist.

### Images or Assets Not Loading

**Cause**: Incorrect paths or missing files in `static/` directory.

**Solution**:
- Place images in `frontend/static/img/`
- Reference as `![Alt text](/img/filename.png)`

### "Page Not Found" on Direct URL Access

**Cause**: Missing rewrite rule for client-side routing.

**Solution**: Already configured in `vercel.json`:
```json
{
  "rewrites": [
    { "source": "/docs", "destination": "/docs/intro" }
  ]
}
```

## Performance Optimization

### Enable Build Cache

Vercel caches dependencies by default. To clear cache:
```bash
vercel --force
```

### Analyze Bundle Size

```bash
cd frontend
ANALYZE=true npm run build
```

### Optimize Images

- Use WebP format for images
- Compress images before adding to `static/img/`
- Use responsive image sizes

## Security Headers

Security headers are already configured in `vercel.json`:

- `X-Content-Type-Options: nosniff`
- `X-Frame-Options: DENY`
- `X-XSS-Protection: 1; mode=block`
- `Referrer-Policy: strict-origin-when-cross-origin`
- `Permissions-Policy: camera=(), microphone=(), geolocation=()`

Verify with [securityheaders.com](https://securityheaders.com/).

## Next Steps

After deploying the frontend:

1. **Set Up Backend**:
   - Deploy FastAPI to Railway
   - Configure Neon Postgres and Qdrant Cloud
   - Implement RAG chatbot endpoints

2. **Integrate Chatbot**:
   - Add chat widget to Docusaurus theme
   - Connect to backend API
   - Test contextual and general RAG

3. **Add Authentication**:
   - Implement Auth.js with Neon Postgres
   - Add sign-up/sign-in pages
   - Store user background questionnaire

4. **Implement Bonus Features**:
   - Content personalization
   - Urdu translation
   - Claude Code skills (`/hardware`, `/ros2`)

---

**Deployment Status**:  Ready for Vercel deployment

**Estimated Deployment Time**: 2-3 minutes

**Vercel Free Tier Limits**:
- 100 GB bandwidth/month (sufficient for textbook site)
- Unlimited deployments
- Automatic HTTPS
- CDN caching globally

**Questions?** Check [Vercel Docusaurus Guide](https://vercel.com/guides/deploying-docusaurus-with-vercel)
